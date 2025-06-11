import modal

app = modal.App("simple-object-detection-test")

@app.function()
@modal.web_endpoint(method="GET")
def hello():
    return {"message": "Object detection test is working!", "status": "ready"}

@app.function()
@modal.web_endpoint(method="GET")
def test_page():
    html = """
    <!DOCTYPE html>
    <html>
    <head><title>Simple Test</title></head>
    <body>
        <h1>Modal Object Detection Test</h1>
        <p>If you can see this page, the Modal app is working correctly!</p>
        <p>The object detection model is being prepared...</p>
    </body>
    </html>
    """
    return modal.Response(html, headers={"Content-Type": "text/html"})

@app.local_entrypoint()
def main():
    print("Simple test app ready!")