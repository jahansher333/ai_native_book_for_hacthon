# GitHub Deployment Guide

## Overview

Complete guide for deploying the Physical AI & Robotics project from GitHub with automated CI/CD pipelines.

## Repository Structure

```
ai_robotics_book/
├── .github/
│   └── workflows/
│       ├── backend-deploy.yml    # Backend CI/CD
│       └── frontend-deploy.yml   # Frontend CI/CD
├── backend/
│   ├── Dockerfile
│   ├── requirements.txt
│   └── src/
├── frontend/
│   ├── package.json
│   └── src/
├── docker-compose.yml
└── README.md
```

## Quick Start

### 1. Push Code to GitHub

```bash
# Initialize git (if not already)
cd "D:\New folder (6)\ai_robotics_book"
git init

# Add all files
git add .

# Commit
git commit -m "Initial commit: Physical AI & Robotics platform"

# Add remote (replace with your repo URL)
git remote add origin https://github.com/YOUR_USERNAME/ai_robotics_book.git

# Push to main branch
git push -u origin main
```

### 2. Set Up GitHub Secrets

Go to: **Settings → Secrets and variables → Actions → New repository secret**

#### Required Secrets:

**Backend:**
```
GEMINI_API_KEY=your_gemini_api_key
GROQ_API_KEY=your_groq_api_key
JWT_SECRET=your_generated_secret (openssl rand -hex 32)
```

**Docker Hub (for backend deployment):**
```
DOCKER_USERNAME=your_dockerhub_username
DOCKER_PASSWORD=your_dockerhub_password
```

**Deployment Platform (choose one):**

**Railway:**
```
RAILWAY_TOKEN=your_railway_token
RAILWAY_PROJECT_ID=your_project_id
```

**Render:**
```
RENDER_API_KEY=your_render_api_key
RENDER_SERVICE_ID=your_service_id
```

**AWS:**
```
AWS_ACCESS_KEY_ID=your_aws_access_key
AWS_SECRET_ACCESS_KEY=your_aws_secret_key
```

**Vercel (optional for frontend):**
```
VERCEL_TOKEN=your_vercel_token
VERCEL_ORG_ID=your_org_id
VERCEL_PROJECT_ID=your_project_id
```

**Netlify (optional for frontend):**
```
NETLIFY_AUTH_TOKEN=your_netlify_token
NETLIFY_SITE_ID=your_site_id
```

### 3. Enable GitHub Actions

1. Go to **Actions** tab in your repository
2. Enable workflows if prompted
3. Workflows will run automatically on push/PR

### 4. Enable GitHub Pages (for frontend)

1. Go to **Settings → Pages**
2. Source: **GitHub Actions**
3. The frontend will be deployed at: `https://YOUR_USERNAME.github.io/ai_robotics_book/`

## CI/CD Workflows

### Backend Workflow (backend-deploy.yml)

**Triggers:**
- Push to `main` or `001-docusaurus-textbook` branches
- Pull requests to these branches
- Changes in `backend/` directory

**Jobs:**

1. **Test** - Run pytest tests
2. **Lint** - Run Black, isort, flake8
3. **Build** - Build Docker image and push to Docker Hub
4. **Deploy** - Deploy to production (Railway/Render/AWS)
5. **Notify** - Send deployment status notification

**Workflow Steps:**
```
Push to GitHub
    ↓
Run Tests (pytest)
    ↓
Lint Code (Black, flake8)
    ↓
Build Docker Image
    ↓
Push to Docker Hub
    ↓
Deploy to Production
    ↓
Notify Status
```

### Frontend Workflow (frontend-deploy.yml)

**Triggers:**
- Push to `main` or `001-docusaurus-textbook` branches
- Pull requests to these branches
- Changes in `frontend/` directory

**Jobs:**

1. **Test** - Lint, type check, build Docusaurus
2. **Deploy GitHub Pages** - Automatic deployment
3. **Deploy Vercel** (optional) - If secrets configured
4. **Deploy Netlify** (optional) - If secrets configured

**Workflow Steps:**
```
Push to GitHub
    ↓
Install Dependencies (npm ci)
    ↓
Run Linter
    ↓
Type Check (TypeScript)
    ↓
Build Docusaurus
    ↓
Deploy to GitHub Pages / Vercel / Netlify
```

## Deployment Options

### Option 1: GitHub Pages (Free, Recommended for Frontend)

**Setup:**
1. Enable GitHub Pages in repository settings
2. Set source to "GitHub Actions"
3. Push code to trigger deployment

**URL:** `https://YOUR_USERNAME.github.io/ai_robotics_book/`

**Pros:**
- ✅ Free hosting
- ✅ Automatic HTTPS
- ✅ CDN included
- ✅ Easy setup

**Cons:**
- ❌ Static sites only (no backend)
- ❌ Limited to public repos (free tier)

### Option 2: Railway (Recommended for Backend)

**Setup:**
```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Link project
railway link

# Get project ID and token
railway whoami
```

**Add to GitHub Secrets:**
```
RAILWAY_TOKEN=your_token
RAILWAY_PROJECT_ID=your_project_id
```

**Pricing:** $5/month for 500 hours

**Pros:**
- ✅ Easy Docker deployment
- ✅ Automatic SSL
- ✅ Postgres/Redis included
- ✅ GitHub integration

### Option 3: Render (Alternative Backend)

**Setup:**
1. Create account at render.com
2. Connect GitHub repository
3. Create Web Service
4. Add environment variables

**Add to GitHub Secrets:**
```
RENDER_API_KEY=your_api_key
RENDER_SERVICE_ID=your_service_id
```

**Pricing:** Free tier available, paid plans from $7/month

**Pros:**
- ✅ Free tier (750 hours/month)
- ✅ Automatic deploys
- ✅ Built-in databases
- ✅ Easy configuration

### Option 4: Vercel (Alternative Frontend)

**Setup:**
```bash
# Install Vercel CLI
npm install -g vercel

# Login and link
vercel login
vercel link
```

**Add to GitHub Secrets:**
```
VERCEL_TOKEN=your_token
VERCEL_ORG_ID=your_org_id
VERCEL_PROJECT_ID=your_project_id
```

**Pricing:** Free for personal projects

**Pros:**
- ✅ Excellent for React/Next.js
- ✅ Edge network
- ✅ Automatic HTTPS
- ✅ Preview deployments

### Option 5: AWS ECS/Fargate (Enterprise)

**Setup:**
```bash
# Configure AWS credentials
aws configure

# Create ECR repository
aws ecr create-repository --repository-name ai-robotics-backend

# Push Docker image
docker build -t ai-robotics-backend ./backend
aws ecr get-login-password | docker login --username AWS --password-stdin <ecr-url>
docker tag ai-robotics-backend:latest <ecr-url>/ai-robotics-backend:latest
docker push <ecr-url>/ai-robotics-backend:latest
```

**Add to GitHub Secrets:**
```
AWS_ACCESS_KEY_ID=your_access_key
AWS_SECRET_ACCESS_KEY=your_secret_key
```

**Pricing:** Pay as you go

**Pros:**
- ✅ Enterprise-grade
- ✅ Full control
- ✅ Scalable
- ✅ AWS ecosystem

## Environment Variables Setup

### Backend Production (.env)

```env
# API Keys
GEMINI_API_KEY=your_gemini_api_key
GROQ_API_KEY=your_groq_api_key

# Database (Use managed service in production)
NEON_DATABASE_URL=postgresql://user:pass@your-neon-endpoint/dbname

# Qdrant (Use Qdrant Cloud)
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=Physical_Robotics_Book

# Application
ENVIRONMENT=production
DEBUG=False
LOG_LEVEL=INFO

# CORS (Update with your frontend URL)
CORS_ORIGINS=https://your-username.github.io,https://your-custom-domain.com

# JWT
JWT_SECRET=your_generated_secret
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7

# RAG Settings
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
TOP_K_RESULTS=5
SIMILARITY_THRESHOLD=0.3
```

### Frontend Production

Update `frontend/docusaurus.config.ts`:

```typescript
const config: Config = {
  url: 'https://your-username.github.io',
  baseUrl: '/ai_robotics_book/',

  // ... rest of config
};
```

## Monitoring Deployments

### View GitHub Actions

1. Go to **Actions** tab
2. Click on latest workflow run
3. View logs for each job

### Check Deployment Status

**Backend:**
```bash
# Check health endpoint
curl https://your-backend-url.com/api/health

# Expected response:
{
  "status": "healthy",
  "gemini_api_available": true,
  "timestamp": 1234567890
}
```

**Frontend:**
```bash
# Visit your site
https://your-username.github.io/ai_robotics_book/

# Check console for errors
```

## Troubleshooting

### Issue: GitHub Actions failing

**Solution:**
```bash
# Check workflow logs in Actions tab
# Common issues:
# - Missing secrets
# - Incorrect secret names
# - Python/Node version mismatch
```

### Issue: Backend health check failing

**Solution:**
```bash
# Verify environment variables
# Check logs in deployment platform
# Ensure API keys are valid
```

### Issue: Docker build timeout

**Solution:**
```yaml
# Increase timeout in workflow
jobs:
  build:
    timeout-minutes: 30  # Increase from default 15
```

### Issue: Frontend build failing

**Solution:**
```bash
# Check Node version (should be 18+)
# Verify package-lock.json is committed
# Clear npm cache: npm ci --cache .npm
```

## Rolling Back Deployments

### GitHub Pages
```bash
# Rollback to previous commit
git revert HEAD
git push origin main
```

### Railway
```bash
# Rollback via CLI
railway rollback

# Or via dashboard: Deployments → Select previous → Redeploy
```

### Render
```bash
# Rollback via API
curl -X POST \
  -H "Authorization: Bearer $RENDER_API_KEY" \
  https://api.render.com/v1/services/$SERVICE_ID/rollback
```

## Security Best Practices

1. **Never commit secrets** - Use GitHub Secrets
2. **Use environment-specific configs** - Separate dev/prod
3. **Enable branch protection** - Require PR reviews
4. **Scan dependencies** - Use Dependabot
5. **Enable 2FA** - On GitHub and deployment platforms
6. **Rotate secrets regularly** - Every 90 days
7. **Use least privilege** - Minimal IAM permissions

## Performance Optimization

### Enable Caching

**Backend:**
```yaml
# In backend-deploy.yml
- name: Cache pip dependencies
  uses: actions/cache@v3
  with:
    path: ~/.cache/pip
    key: ${{ runner.os }}-pip-${{ hashFiles('backend/requirements.txt') }}
```

**Frontend:**
```yaml
# In frontend-deploy.yml
- name: Cache node modules
  uses: actions/cache@v3
  with:
    path: frontend/node_modules
    key: ${{ runner.os }}-node-${{ hashFiles('frontend/package-lock.json') }}
```

### Parallel Jobs

```yaml
jobs:
  backend:
    # ...
  frontend:
    # ...
  # Both run in parallel by default
```

## Cost Estimation

### Free Tier Setup:
- GitHub Actions: 2,000 minutes/month (free)
- GitHub Pages: Unlimited (free for public repos)
- Railway: $5/month (500 hours)
- **Total: ~$5/month**

### Paid Setup:
- Railway (backend): $5-20/month
- Vercel (frontend): Free - $20/month
- Neon (PostgreSQL): $19/month
- Qdrant Cloud: $25/month
- **Total: ~$50-85/month**

## Next Steps

1. ✅ Push code to GitHub
2. ✅ Add GitHub Secrets
3. ✅ Enable GitHub Actions
4. ✅ Configure deployment platform
5. ✅ Test deployment
6. ✅ Set up custom domain (optional)
7. ✅ Configure monitoring (optional)

## Summary

You now have:
- ✅ Automated CI/CD pipelines for backend and frontend
- ✅ Docker-based backend deployment
- ✅ GitHub Pages deployment for frontend
- ✅ Multiple deployment platform options
- ✅ Comprehensive deployment documentation

**Ready for production deployment!** 🚀

### Quick Deploy Commands:
```bash
# Push to GitHub
git add .
git commit -m "Deploy to production"
git push origin main

# GitHub Actions will automatically:
# 1. Test code
# 2. Build Docker image
# 3. Deploy backend to Railway/Render
# 4. Deploy frontend to GitHub Pages
```
