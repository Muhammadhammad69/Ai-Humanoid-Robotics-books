#!/bin/bash

# Script to check Qdrant collection stats
# This script will verify that embeddings were successfully stored in Qdrant

echo "🔍 Checking Qdrant collection stats..."
echo ""

# Check if we're in the right directory
if [ ! -f "check_qdrant_stats.py" ]; then
    echo "❌ Script not found in current directory!"
    echo "Please run this from the backend directory where check_qdrant_stats.py is located."
    exit 1
fi

# Check if Python is available
if ! command -v python &> /dev/null; then
    echo "❌ Python is not installed or not in PATH"
    exit 1
fi

# Run the Python script
python check_qdrant_stats.py

exit 0