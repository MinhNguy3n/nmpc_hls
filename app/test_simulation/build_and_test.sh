#!/bin/bash
# ==============================================================
# Build and Run Script for XPso_fsm Driver Test
# ==============================================================
#
# This script builds and runs the test_xpso_fsm program.
# The test validates the PSO FSM driver functions for the
# FPGA-based optimization of drone control inputs.
#
# Usage:
#   ./build_and_test.sh [build|run|clean|all]
#
# ==============================================================

set -e

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$SCRIPT_DIR/build"
BIN_DIR="$BUILD_DIR/bin"
EXECUTABLE="$BIN_DIR/test_xpso_fsm"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Functions
print_header() {
    echo ""
    echo -e "${BLUE}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

build_project() {
    print_header "Building XPso_fsm Test Suite"

    if [ ! -f "$SCRIPT_DIR/Makefile.test" ]; then
        print_error "Makefile.test not found in $SCRIPT_DIR"
        return 1
    fi

    print_info "Building with: make -f Makefile.test build"
    cd "$SCRIPT_DIR"

    if make -f Makefile.test build; then
        print_success "Build completed successfully"
        return 0
    else
        print_error "Build failed"
        return 1
    fi
}

run_test() {
    print_header "Running XPso_fsm Test Suite"

    if [ ! -f "$EXECUTABLE" ]; then
        print_error "Test executable not found: $EXECUTABLE"
        print_info "Please run 'build' first"
        return 1
    fi

    print_info "Executing: $EXECUTABLE"
    echo ""

    if "$EXECUTABLE"; then
        echo ""
        print_success "Test execution completed"
        return 0
    else
        print_error "Test execution failed"
        return 1
    fi
}

clean_project() {
    print_header "Cleaning Build Artifacts"

    cd "$SCRIPT_DIR"
    if make -f Makefile.test clean; then
        print_success "Clean completed successfully"
        return 0
    else
        print_error "Clean failed"
        return 1
    fi
}

show_usage() {
    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║  XPso_fsm Driver Test - Build and Run Script                  ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build       Build the test executable"
    echo "  run         Run the test (requires build first)"
    echo "  clean       Remove build artifacts"
    echo "  all         Build and run tests (default)"
    echo "  help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 all       # Build and run everything"
    echo "  $0 build     # Just build the executable"
    echo "  $0 run       # Just run the test"
    echo "  $0 clean     # Clean up build files"
    echo ""
    echo "Output:"
    echo "  Executable: $EXECUTABLE"
    echo "  Build dir:  $BUILD_DIR"
    echo ""
}

# Main logic
main() {
    local command="${1:-all}"

    case "$command" in
        build)
            build_project
            ;;
        run)
            run_test
            ;;
        clean)
            clean_project
            ;;
        all)
            if build_project; then
                run_test
            fi
            ;;
        help)
            show_usage
            ;;
        *)
            print_error "Unknown command: $command"
            show_usage
            exit 1
            ;;
    esac
}

# Run main
main "$@"
