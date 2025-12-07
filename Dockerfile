# Use Python 3.11 slim image
FROM python:3.11-slim

# Install system dependencies including git and build tools
RUN apt-get update && apt-get install -y \
    git \
    gcc \
    g++ \
    build-essential \
    libssl-dev \
    libffi-dev \
    python3-dev \
    cargo \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy backend requirements first for better caching
COPY backend/requirements.txt /app/backend/requirements.txt

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r /app/backend/requirements.txt

# Copy entire backend directory
COPY backend /app/backend

# Set working directory to backend
WORKDIR /app/backend

# Make start script executable
RUN chmod +x start.sh

# Expose port (Railway will set $PORT dynamically)
EXPOSE 8000

# Start command using shell script for PORT variable support
CMD ["./start.sh"]
