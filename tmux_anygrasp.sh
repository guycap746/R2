#!/bin/bash

# AnyGrasp tmux session manager
set -euo pipefail

readonly SESSION_NAME="anygrasp"
readonly PROJECT_DIR="/root/ros2_workspace"
readonly SCRIPT_PATH="$PROJECT_DIR/run_anygrasp_claude_v2.sh"

# Colors
readonly GREEN='\033[0;32m'
readonly RED='\033[0;31m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

# Check if session exists
session_exists() {
    tmux has-session -t "$SESSION_NAME" 2>/dev/null
}

# Start new session
start_session() {
    if session_exists; then
        log_warn "Session '$SESSION_NAME' already exists"
        log_info "Use 'tr' to attach or 'tk' to kill first"
        return 1
    fi
    
    log_info "Starting new tmux session: $SESSION_NAME"
    tmux new-session -d -s "$SESSION_NAME" -c "$PROJECT_DIR"
    tmux send-keys -t "$SESSION_NAME" "cd $PROJECT_DIR && $SCRIPT_PATH" Enter
    tmux attach -t "$SESSION_NAME"
}

# Attach to existing session
attach_session() {
    if ! session_exists; then
        log_error "No session '$SESSION_NAME' found"
        log_info "Use 't' to start a new session"
        return 1
    fi
    
    log_info "Attaching to session: $SESSION_NAME"
    tmux attach -t "$SESSION_NAME"
}

# Kill session
kill_session() {
    if ! session_exists; then
        log_warn "No session '$SESSION_NAME' to kill"
        return 0
    fi
    
    log_info "Killing session: $SESSION_NAME"
    tmux kill-session -t "$SESSION_NAME"
    log_info "Session killed"
}

# List sessions
list_sessions() {
    log_info "Current tmux sessions:"
    tmux list-sessions 2>/dev/null || log_warn "No tmux sessions found"
}

# Show session status
show_status() {
    if session_exists; then
        log_info "Session '$SESSION_NAME' is ACTIVE"
        tmux display-message -t "$SESSION_NAME" -p "Session: #S | Window: #W | Pane: #P | Time: #{session_created_string}"
    else
        log_info "Session '$SESSION_NAME' is NOT ACTIVE"
    fi
}

# Main function
main() {
    case "${1:-start}" in
        start|s)
            start_session
            ;;
        attach|a)
            attach_session
            ;;
        kill|k)
            kill_session
            ;;
        status|st)
            show_status
            ;;
        list|l)
            list_sessions
            ;;
        help|h)
            cat << EOF
AnyGrasp tmux session manager

Usage: $0 [COMMAND]

Commands:
  start, s    Start new session (default)
  attach, a   Attach to existing session
  kill, k     Kill session
  status, st  Show session status
  list, l     List all sessions
  help, h     Show this help

Aliases:
  t   - Start session
  tr  - Attach to session
  tk  - Kill session
  tl  - List sessions
EOF
            ;;
        *)
            log_error "Unknown command: $1"
            log_info "Use '$0 help' for usage information"
            return 1
            ;;
    esac
}

main "$@"