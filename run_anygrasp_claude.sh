#!/bin/bash

# Enhanced logging and error handling for AnyGrasp Claude processing
set -euo pipefail

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOGDIR="./logs"
LOGFILE="$LOGDIR/claude_log_$TIMESTAMP.json"
SCRIPTLOG="$LOGDIR/script_log_$TIMESTAMP.log"

# Create logs directory
mkdir -p "$LOGDIR"

# Logging functions
log_info() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] INFO: $1" | tee -a "$SCRIPTLOG"
}

log_error() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] ERROR: $1" | tee -a "$SCRIPTLOG" >&2
}

log_success() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] SUCCESS: $1" | tee -a "$SCRIPTLOG"
}

# Cleanup function
cleanup() {
    log_info "Cleaning up temporary files..."
}

# Error handler
error_handler() {
    local exit_code=$?
    log_error "Script failed with exit code $exit_code on line $1"
    cleanup
    exit $exit_code
}

# Set error trap
trap 'error_handler $LINENO' ERR
trap cleanup EXIT

# Start logging
log_info "Starting AnyGrasp Claude processing session"
log_info "Timestamp: $TIMESTAMP"
log_info "Claude log file: $LOGFILE"
log_info "Script log file: $SCRIPTLOG"

# Verify files exist
if [[ ! -f "/root/ros2_workspace/ANYGRASP_IMPROVEMENT_TASKS.md" ]]; then
    log_error "ANYGRASP_IMPROVEMENT_TASKS.md not found"
    exit 1
fi

log_info "Found ANYGRASP_IMPROVEMENT_TASKS.md file"

# Check if claude command exists
if ! command -v claude &> /dev/null; then
    log_error "Claude command not found in PATH"
    exit 1
fi

log_info "Starting Claude Code session..."

# Run Claude with enhanced logging
claude code \
  --interactive \
  --enable-execution \
  --enable-file-system \
  --enable-network \
  --enable-shell \
  --project-dir /root/ros2_workspace \
  --log-format json \
  --log-file "$LOGFILE" \
  --trusted \
  /root/ros2_workspace/ANYGRASP_IMPROVEMENT_TASKS.md

# Check if Claude session completed successfully
if [[ $? -eq 0 ]]; then
    log_success "Claude session completed successfully"
    log_info "Logs available at: $LOGFILE"
    log_info "Script logs at: $SCRIPTLOG"
else
    log_error "Claude session failed"
    exit 1
fi