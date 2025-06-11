import base64
import io
from pathlib import Path
import modal

MODEL_REPO_ID = "facebook/detr-resnet-50"
MODEL_DIR = "/cache"

app = modal.App("webcam-test-with-samples")
image = (
    modal.Image.debian_slim(python_version="3.12")
    .pip_install(
        "huggingface-hub==0.27.1",
        "Pillow",
        "timm",
        "transformers",
    )
    .apt_install("fonts-freefont-ttf")
    .env({"HF_HUB_CACHE": MODEL_DIR})
)

with image.imports():
    import torch
    from huggingface_hub import snapshot_download
    from PIL import Image, ImageColor, ImageDraw, ImageFont
    from transformers import DetrForObjectDetection, DetrImageProcessor

cache_volume = modal.Volume.from_name("hf-hub-cache", create_if_missing=True)

@app.function(image=image, volumes={MODEL_DIR: cache_volume})
def download_model():
    loc = snapshot_download(repo_id=MODEL_REPO_ID)
    print(f"Saved model to {loc}")

@app.cls(image=image, volumes={MODEL_DIR: cache_volume})
class ObjectDetection:
    @modal.enter()
    def load_model(self):
        self.feature_extractor = DetrImageProcessor.from_pretrained(MODEL_REPO_ID)
        self.model = DetrForObjectDetection.from_pretrained(MODEL_REPO_ID)

    @modal.method()
    def detect(self, img_data_in):
        image = Image.open(io.BytesIO(img_data_in)).convert("RGB")
        inputs = self.feature_extractor(image, return_tensors="pt")
        outputs = self.model(**inputs)
        img_size = torch.tensor([tuple(reversed(image.size))])
        processed_outputs = self.feature_extractor.post_process_object_detection(
            outputs=outputs,
            target_sizes=img_size,
            threshold=0,
        )
        output_dict = processed_outputs[0]

        keep = output_dict["scores"] > 0.7
        boxes = output_dict["boxes"][keep].tolist()
        scores = output_dict["scores"][keep].tolist()
        labels = output_dict["labels"][keep].tolist()

        colors = list(ImageColor.colormap.values())
        font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 18)
        output_image = Image.new("RGBA", (image.width, image.height))
        output_image_draw = ImageDraw.Draw(output_image)
        
        for _score, box, label in zip(scores, boxes, labels):
            color = colors[label % len(colors)]
            text = self.model.config.id2label[label]
            box = tuple(map(int, box))
            output_image_draw.rectangle(box, outline=color)
            output_image_draw.text(box[:2], text, font=font, fill=color, width=3)

        with io.BytesIO() as output_buf:
            output_image.save(output_buf, format="PNG")
            return output_buf.getvalue()

@app.function(
    image=modal.Image.debian_slim(python_version="3.12")
    .pip_install("fastapi[standard]==0.115.4")
)
@modal.asgi_app(label="webcam-test-with-samples")
def fastapi_app():
    from fastapi import FastAPI, Request, Response
    from fastapi.responses import HTMLResponse

    web_app = FastAPI()

    # Sample test images (base64 encoded)
    sample_images = {
        "people": "https://images.unsplash.com/photo-1529258283598-8d6fe60b27f4?w=640&h=480&fit=crop",
        "room": "https://images.unsplash.com/photo-1586023492125-27b2c045efd7?w=640&h=480&fit=crop",
        "street": "https://images.unsplash.com/photo-1449824913935-59a10b8d2000?w=640&h=480&fit=crop"
    }

    @web_app.get("/", response_class=HTMLResponse)
    async def root():
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Object Detection Test with Sample Images</title>
            <style>
                body {{ font-family: Arial, sans-serif; text-align: center; margin: 20px; }}
                .sample-button {{ padding: 10px 20px; margin: 5px; font-size: 16px; background: #4CAF50; color: white; border: none; cursor: pointer; }}
                .sample-button:hover {{ background: #45a049; }}
                #result {{ margin: 20px auto; max-width: 640px; }}
                #loading {{ display: none; color: #666; }}
            </style>
        </head>
        <body>
            <h1>Object Detection Test</h1>
            <p>Click on a sample image to test object detection:</p>
            
            <div>
                <button class="sample-button" onclick="testSample('people')">Test with People</button>
                <button class="sample-button" onclick="testSample('room')">Test with Room</button>
                <button class="sample-button" onclick="testSample('street')">Test with Street</button>
            </div>
            
            <div id="loading">Processing... Please wait</div>
            <div id="result"></div>

            <script>
                async function testSample(type) {{
                    document.getElementById('loading').style.display = 'block';
                    document.getElementById('result').innerHTML = '';
                    
                    try {{
                        // Create a canvas to convert the image URL to base64
                        const img = new Image();
                        img.crossOrigin = 'anonymous';
                        
                        const urls = {{
                            'people': '{sample_images["people"]}',
                            'room': '{sample_images["room"]}', 
                            'street': '{sample_images["street"]}'
                        }};
                        
                        img.onload = async function() {{
                            const canvas = document.createElement('canvas');
                            const ctx = canvas.getContext('2d');
                            canvas.width = img.width;
                            canvas.height = img.height;
                            ctx.drawImage(img, 0, 0);
                            
                            const dataURL = canvas.toDataURL('image/png');
                            
                            // Send to prediction endpoint
                            const response = await fetch('/predict', {{
                                method: 'POST',
                                headers: {{ 'Content-Type': 'text/plain' }},
                                body: dataURL
                            }});
                            
                            const resultData = await response.text();
                            document.getElementById('loading').style.display = 'none';
                            document.getElementById('result').innerHTML = `
                                <h3>Original Image:</h3>
                                <img src="${{dataURL}}" style="max-width: 320px; border: 2px solid #ddd;">
                                <h3>Detection Result:</h3>
                                <img src="${{resultData}}" style="max-width: 320px; border: 2px solid #ddd;">
                            `;
                        }};
                        
                        img.onerror = function() {{
                            document.getElementById('loading').style.display = 'none';
                            document.getElementById('result').innerHTML = '<p style="color: red;">Error loading sample image</p>';
                        }};
                        
                        img.src = urls[type];
                        
                    }} catch (error) {{
                        document.getElementById('loading').style.display = 'none';
                        document.getElementById('result').innerHTML = '<p style="color: red;">Error: ' + error.message + '</p>';
                    }}
                }}
            </script>
        </body>
        </html>
        """
        return html

    @web_app.post("/predict")
    async def predict(request: Request):
        body = await request.body()
        img_data_in = base64.b64decode(body.split(b",")[1])
        img_data_out = ObjectDetection().detect.remote(img_data_in)
        output_data = b"data:image/png;base64," + base64.b64encode(img_data_out)
        return Response(content=output_data)

    return web_app

@app.local_entrypoint()
def main():
    print("Downloading model...")
    download_model.remote()
    print("Model downloaded. Starting test server...")