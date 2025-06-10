#!/bin/bash

# Enhanced AnyGrasp Claude processing with improved error handling
set -euo pipefail

# Configuration
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_DIR="${PROJECT_DIR:-$SCRIPT_DIR}"
readonly TASK_FILE="${TASK_FILE:-$PROJECT_DIR/ANYGRASP_IMPROVEMENT_TASKS.md}"
readonly TIMESTAMP=$(date +%Y%m%d_%H%M%S)
readonly LOGDIR="$PROJECT_DIR/logs"
readonly LOGFILE="$LOGDIR/claude_log_$TIMESTAMP.json"
readonly SCRIPTLOG="$LOGDIR/script_log_$TIMESTAMP.log"
readonly PIDFILE="$LOGDIR/claude_session.pid"

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m' # No Color

# Create logs directory
mkdir -p "$LOGDIR"

# Enhanced logging functions
log_with_color() {
    local color=$1
    local level=$2
    local message=$3
    echo -e "${color}[$(date '+%Y-%m-%d %H:%M:%S')] $level: $message${NC}" | tee -a "$SCRIPTLOG"
}

log_info() { log_with_color "$BLUE" "INFO" "$1"; }
log_warn() { log_with_color "$YELLOW" "WARN" "$1"; }
log_error() { log_with_color "$RED" "ERROR" "$1" >&2; }
log_success() { log_with_color "$GREEN" "SUCCESS" "$1"; }

# Performance tracking
start_time=$(date +%s)
log_performance() {
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    log_info "Session duration: ${duration}s"
}

# Cleanup function
cleanup() {
    log_info "Cleaning up..."
    [[ -f "$PIDFILE" ]] && rm -f "$PIDFILE"
    log_performance
}

# Enhanced error handler
error_handler() {
    local exit_code=$?
    local line_no=$1
    log_error "Script failed with exit code $exit_code on line $line_no"
    log_error "Command: ${BASH_COMMAND}"
    cleanup
    exit $exit_code
}

# Signal handlers
handle_interrupt() {
    log_warn "Received interrupt signal"
    cleanup
    exit 130
}

# Set traps
trap 'error_handler $LINENO' ERR
trap cleanup EXIT
trap handle_interrupt INT TERM

# Validation functions
validate_environment() {
    log_info "Validating environment..."
    
    # Check if running in tmux
    if [[ -n "${TMUX:-}" ]]; then
        log_info "Running in tmux session: $(tmux display-message -p '#S')"
    fi
    
    # Check task file
    if [[ ! -f "$TASK_FILE" ]]; then
        log_error "Task file not found: $TASK_FILE"
        return 1
    fi
    log_info "Found task file: $TASK_FILE"
    
    # Check claude command
    if ! command -v claude &> /dev/null; then
        log_error "Claude command not found in PATH"
        return 1
    fi
    
    # Check for existing session
    if [[ -f "$PIDFILE" ]]; then
        local old_pid=$(cat "$PIDFILE")
        if kill -0 "$old_pid" 2>/dev/null; then
            log_warn "Existing Claude session found (PID: $old_pid)"
            read -p "Kill existing session? [y/N]: " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                kill "$old_pid" && log_info "Killed existing session"
            else
                log_error "Cannot start new session while another is running"
                return 1
            fi
        fi
    fi
    
    log_success "Environment validation passed"
}

# Main execution
main() {
    log_info "Starting AnyGrasp Claude processing session"
    log_info "Project directory: $PROJECT_DIR"
    log_info "Claude log: $LOGFILE"
    log_info "Script log: $SCRIPTLOG"
    
    validate_environment
    
    # Store PID for session management
    echo $$ > "$PIDFILE"
    
    log_info "Starting Claude Code session..."
    
    # Run Claude with supported flags
    claude code \
        --debug \
        "$TASK_FILE"
    
    # Capture exit code properly
    local claude_exit_code=$?
    
    if [[ $claude_exit_code -eq 0 ]]; then
        log_success "Claude session completed successfully"
        log_info "Logs available at: $LOGFILE"
        log_info "Script logs at: $SCRIPTLOG"
    else
        log_error "Claude session failed with exit code: $claude_exit_code"
        return $claude_exit_code
    fi
}

# Usage information
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Environment Variables:
  PROJECT_DIR    Project directory (default: script directory)
  TASK_FILE      Task file path (default: PROJECT_DIR/ANYGRASP_IMPROVEMENT_TASKS.md)

Options:
  -h, --help     Show this help message

Examples:
  $0                                    # Use defaults
  PROJECT_DIR=/custom/path $0           # Custom project directory
  TASK_FILE=/path/to/tasks.md $0        # Custom task file
EOF
}

# Parse arguments
case "${1:-}" in
    -h|--help)
        show_usage
        exit 0
        ;;
    *)
        main "$@"
        ;;
esac