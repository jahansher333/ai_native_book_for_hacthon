#!/bin/bash
# Start script for Railway deployment
# Uses PORT environment variable or defaults to 8000

PORT=${PORT:-8000}
echo "Starting uvicorn on port $PORT..."
exec uvicorn src.main:app --host 0.0.0.0 --port $PORT
