# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Copy backend requirements first for better caching
COPY backend/requirements.txt /app/backend/requirements.txt

# Install dependencies
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r /app/backend/requirements.txt

# Copy entire backend directory
COPY backend /app/backend

# Set working directory to backend
WORKDIR /app/backend

# Expose port (Railway will set $PORT dynamically)
EXPOSE 8000

# Start command
CMD uvicorn src.main:app --host 0.0.0.0 --port ${PORT:-8000}
