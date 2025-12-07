# Docker Deployment Guide - Backend

## Overview

This guide covers deploying the Physical AI & Robotics backend using Docker.

## Files Created

1. **Dockerfile** - Backend container definition
2. **.dockerignore** - Files to exclude from Docker build
3. **docker-compose.yml** - Multi-container orchestration (root directory)

## Prerequisites

- Docker 20.10+ installed
- Docker Compose 2.0+ installed
- API keys for:
  - Gemini API (Google AI Studio)
  - Groq API (for LiteLLM agents)
  - Qdrant Cloud (or use local container)

## Quick Start

### 1. Set Environment Variables

Create `.env` file in the root directory:

```bash
# Copy from backend/.env.example
cp backend/.env.example .env

# Edit .env with your API keys
nano .env
```

**Required Variables:**
```env
# Gemini API
GEMINI_API_KEY=your_gemini_api_key
GROQ_API_KEY=your_groq_api_key

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET=your_generated_secret_key

# Qdrant (optional - uses local container if not provided)
QDRANT_URL=http://qdrant:6333
QDRANT_API_KEY=
```

### 2. Build and Run

```bash
# Build and start all services
docker-compose up -d

# View logs
docker-compose logs -f backend

# Stop services
docker-compose down

# Stop and remove volumes (clean slate)
docker-compose down -v
```

## Service URLs

- **Backend API**: http://localhost:8001
- **API Docs (Swagger)**: http://localhost:8001/docs
- **PostgreSQL**: localhost:5432
- **Qdrant**: http://localhost:6333/dashboard
- **Frontend** (if enabled): http://localhost:3000

## Docker Architecture

### Services

1. **backend** (FastAPI + LiteLLM/Groq)
   - Port: 8001
   - Health check every 30s
   - Auto-restart enabled

2. **postgres** (PostgreSQL 15)
   - Port: 5432
   - Persistent volume: `postgres-data`
   - Default credentials: postgres/postgres

3. **qdrant** (Vector Database)
   - Ports: 6333 (HTTP), 6334 (gRPC)
   - Persistent volume: `qdrant-data`
   - Web UI: http://localhost:6333/dashboard

4. **frontend** (Optional - Docusaurus)
   - Port: 3000
   - Hot reload enabled

## Backend Dockerfile Details

```dockerfile
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc g++ libpq-dev curl

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create non-root user
RUN useradd -m -u 1000 appuser
USER appuser

# Run FastAPI
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8001"]
```

**Key Features:**
- Python 3.11 (as per constitution)
- Non-root user for security
- Health check endpoint
- Layer caching for faster builds

## Production Deployment

### Option 1: Cloud Provider (Railway, Render, Fly.io)

#### Railway:
```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Deploy backend
cd backend
railway init
railway up
```

#### Render:
1. Connect GitHub repo
2. Create Web Service
3. Build command: `pip install -r requirements.txt`
4. Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables

### Option 2: AWS ECS/Fargate

```bash
# Build and push to ECR
aws ecr create-repository --repository-name ai-robotics-backend

# Tag and push
docker build -t ai-robotics-backend ./backend
docker tag ai-robotics-backend:latest <ecr-url>/ai-robotics-backend:latest
docker push <ecr-url>/ai-robotics-backend:latest

# Create ECS task definition and service
aws ecs create-task-definition --cli-input-json file://ecs-task-def.json
aws ecs create-service --cluster ai-robotics --service-name backend ...
```

### Option 3: Google Cloud Run

```bash
# Build and deploy
gcloud builds submit --tag gcr.io/PROJECT_ID/ai-robotics-backend ./backend
gcloud run deploy backend \
  --image gcr.io/PROJECT_ID/ai-robotics-backend \
  --platform managed \
  --region us-central1 \
  --allow-unauthenticated
```

## Environment Variables Reference

### Required

| Variable | Description | Example |
|----------|-------------|---------|
| `GEMINI_API_KEY` | Google Gemini API key | `AIza...` |
| `GROQ_API_KEY` | Groq API key for LiteLLM | `gsk_...` |
| `JWT_SECRET` | Secret for JWT tokens | Generated with `openssl rand -hex 32` |

### Optional (with defaults)

| Variable | Default | Description |
|----------|---------|-------------|
| `QDRANT_URL` | `http://qdrant:6333` | Qdrant instance URL |
| `NEON_DATABASE_URL` | `postgresql://postgres:postgres@postgres:5432/ai_robotics` | PostgreSQL connection |
| `CORS_ORIGINS` | `http://localhost:3000` | Allowed CORS origins |
| `CHUNK_SIZE` | `1000` | Text chunk size for RAG |
| `TOP_K_RESULTS` | `5` | Number of RAG results |
| `JWT_EXPIRATION_DAYS` | `7` | JWT token expiration |

## Database Migrations

The backend auto-creates tables on startup. For production, use Alembic:

```bash
# Inside backend container
docker-compose exec backend bash

# Create migration
alembic revision --autogenerate -m "description"

# Apply migration
alembic upgrade head
```

## Monitoring & Logs

### View Logs
```bash
# All services
docker-compose logs -f

# Backend only
docker-compose logs -f backend

# Last 100 lines
docker-compose logs --tail=100 backend
```

### Health Check
```bash
# Check backend health
curl http://localhost:8001/api/health

# Response:
# {
#   "status": "healthy",
#   "gemini_api_available": true,
#   "timestamp": 1234567890
# }
```

### Container Stats
```bash
# Resource usage
docker stats

# Inspect container
docker inspect ai-robotics-backend
```

## Troubleshooting

### Issue: Backend container exits immediately

**Solution:**
```bash
# Check logs
docker-compose logs backend

# Common causes:
# - Missing .env file
# - Invalid API keys
# - Port already in use
```

### Issue: Database connection fails

**Solution:**
```bash
# Verify postgres is running
docker-compose ps postgres

# Check database URL
docker-compose exec backend env | grep DATABASE

# Recreate database
docker-compose down -v postgres
docker-compose up -d postgres
```

### Issue: Qdrant connection fails

**Solution:**
```bash
# Use local Qdrant container
QDRANT_URL=http://qdrant:6333

# Or use Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_api_key
```

### Issue: Permission denied errors

**Solution:**
```bash
# Fix ownership (if volume mounted)
docker-compose exec backend chown -R appuser:appuser /app
```

## Security Best Practices

1. **Never commit `.env` files** - Use `.env.example` as template
2. **Use secrets management** - AWS Secrets Manager, GCP Secret Manager
3. **Scan images** - `docker scan ai-robotics-backend`
4. **Update base images** - Regularly update `python:3.11-slim`
5. **Limit container resources** - Set CPU/memory limits in docker-compose

## Performance Optimization

### Multi-stage Build (Production)

Create `Dockerfile.prod`:

```dockerfile
# Build stage
FROM python:3.11-slim as builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --user --no-cache-dir -r requirements.txt

# Runtime stage
FROM python:3.11-slim
WORKDIR /app
COPY --from=builder /root/.local /root/.local
COPY . .

ENV PATH=/root/.local/bin:$PATH
USER appuser
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8001", "--workers", "4"]
```

**Benefits:**
- Smaller final image (~200MB vs ~500MB)
- No build tools in production
- Faster deployment

### Enable Caching

```bash
# Use BuildKit for better caching
DOCKER_BUILDKIT=1 docker build -t ai-robotics-backend ./backend
```

## Testing Docker Build

```bash
# Build backend only
docker build -t ai-robotics-backend ./backend

# Run backend container standalone
docker run -d \
  --name backend \
  -p 8001:8001 \
  --env-file .env \
  ai-robotics-backend

# Test API
curl http://localhost:8001/api/health

# Stop and remove
docker stop backend && docker rm backend
```

## Scaling (Production)

### Horizontal Scaling

```yaml
# docker-compose.prod.yml
services:
  backend:
    deploy:
      replicas: 3
      resources:
        limits:
          cpus: '1'
          memory: 1G
        reservations:
          cpus: '0.5'
          memory: 512M
```

### Load Balancer (Nginx)

```nginx
upstream backend {
    server backend:8001;
    server backend-2:8001;
    server backend-3:8001;
}

server {
    listen 80;
    location / {
        proxy_pass http://backend;
    }
}
```

## CI/CD Integration

### GitHub Actions

```yaml
# .github/workflows/deploy.yml
name: Deploy Backend

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Build Docker image
        run: docker build -t ai-robotics-backend ./backend

      - name: Push to registry
        run: |
          echo ${{ secrets.DOCKER_PASSWORD }} | docker login -u ${{ secrets.DOCKER_USERNAME }} --password-stdin
          docker push ai-robotics-backend
```

## Summary

✅ **Created Docker setup** with:
- Dockerfile for backend (Python 3.11, FastAPI, LiteLLM)
- .dockerignore for optimized builds
- docker-compose.yml for local development
- Complete deployment documentation

**Ready to deploy!** 🚀

### Quick Commands:
```bash
# Start everything
docker-compose up -d

# View logs
docker-compose logs -f backend

# Stop everything
docker-compose down
```
