# Railway Quick Start Guide

## ✅ Repository is Ready!

Your code has been pushed to: https://github.com/jahansher333/ai_native_book_for_hacthon.git

All Railway configuration files are in place at the root level:
- ✅ `Procfile` - Start command
- ✅ `nixpacks.toml` - Python 3.11 config
- ✅ `railway.json` - Build/deploy settings
- ✅ `railway.toml` - Additional Railway config

## Step-by-Step Deployment

### 1. Create Railway Project

1. Go to https://railway.app/new
2. Click **"Deploy from GitHub repo"**
3. **Authorize Railway** to access your GitHub account
4. Select repository: **`ai_native_book_for_hacthon`**
5. Click **"Deploy Now"**

### 2. Configure Environment Variables

After deployment starts, click on your service, then go to **Variables** tab:

```bash
# Required Variables
GEMINI_API_KEY=your_gemini_api_key_here
GROQ_API_KEY=your_groq_api_key_here
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
JWT_SECRET=your_strong_secret_key_at_least_32_characters

# Optional Variables
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO
CORS_ORIGINS=http://localhost:3000,https://your-frontend-url.vercel.app
MODEL_NAME=gemini-2.0-flash-exp
BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
```

**Note:** Railway automatically sets the `PORT` variable - don't override it!

### 3. Watch Deployment

1. Go to **"Deployments"** tab
2. Click on the latest deployment
3. Click **"View Logs"** to monitor progress
4. Wait for: ✅ **"Build successful"** and ✅ **"Deployment live"**

### 4. Get Your Backend URL

1. Go to **"Settings"** tab
2. Scroll down to **"Networking"** section
3. Click **"Generate Domain"**
4. Your URL will be: `https://your-project-name.up.railway.app`

### 5. Test Your Deployment

Open these URLs in your browser:

```
https://your-project-name.up.railway.app/
https://your-project-name.up.railway.app/api/health
https://your-project-name.up.railway.app/docs
```

All should return successful responses!

## Troubleshooting

### Build Fails with "Cannot find module"

**Solution:** Railway should automatically detect Python from nixpacks.toml. If not:
1. Go to Settings → Build
2. Ensure **Builder** is set to "NIXPACKS"
3. Redeploy

### Port Binding Error

**Solution:** Make sure you didn't set a `PORT` environment variable. Railway sets this automatically.

### Database Connection Failed

**Check:**
1. NEON_DATABASE_URL format is correct
2. Includes `?sslmode=require` at the end
3. Neon database allows external connections

### Qdrant Connection Failed

**Check:**
1. QDRANT_URL is the full URL (https://...)
2. QDRANT_API_KEY is correct
3. Qdrant cluster is active

### Application Crashes on Start

**Check logs for:**
1. Missing environment variables
2. Import errors (check requirements.txt)
3. Database initialization errors

**View logs:**
```bash
# In Railway dashboard
Deployments → Click deployment → View Logs

# Or use Railway CLI
railway logs
```

## Update Frontend to Use Railway Backend

After successful deployment, update your frontend:

1. Edit `frontend/src/config.ts`:
```typescript
export const API_BASE_URL = process.env.REACT_APP_API_URL || 'https://your-project-name.up.railway.app';
```

2. Redeploy frontend to Vercel

3. Update CORS in Railway:
```bash
CORS_ORIGINS=http://localhost:3000,https://your-frontend.vercel.app,https://your-project-name.up.railway.app
```

## Railway CLI (Optional)

Install for easier management:

```bash
npm i -g @railway/cli
railway login
railway link
railway logs          # View logs
railway open          # Open in browser
railway variables     # Manage environment variables
```

## Auto-Deploy on Git Push

Railway automatically redeploys when you push to GitHub:

1. Make changes locally
2. Commit: `git commit -m "your message"`
3. Push: `git push`
4. Railway detects changes and redeploys automatically

## Production Checklist

Before going live:

- [ ] All environment variables set correctly
- [ ] JWT_SECRET is strong and unique (32+ characters)
- [ ] CORS_ORIGINS includes all frontend domains
- [ ] Database tables created successfully (check logs)
- [ ] Qdrant collection created (check logs)
- [ ] Health endpoint returns 200 OK
- [ ] API documentation accessible at /docs
- [ ] Frontend can connect to backend
- [ ] Authentication flows work (signup/signin)
- [ ] Query endpoint returns results
- [ ] No errors in Railway logs

## Cost Optimization

Railway offers:
- **$5 free credit** monthly for Hobby plan
- **Pay per usage** after free credit

To monitor costs:
1. Go to Settings → Usage
2. Set up spending alerts
3. Monitor resource usage

## Support & Resources

- **Railway Docs:** https://docs.railway.app
- **Railway Discord:** https://discord.gg/railway
- **Your Deployment Guide:** See `backend/RAILWAY_DEPLOYMENT.md`
- **Project Issues:** https://github.com/jahansher333/ai_native_book_for_hacthon/issues

## Common Railway Commands

```bash
# Link to project
railway link

# View logs
railway logs

# Open dashboard
railway open

# View variables
railway variables

# Add variable
railway variables set KEY=value

# Deploy manually
railway up

# View deployments
railway status
```

## What Railway Does Automatically

✅ Detects Python from `nixpacks.toml`
✅ Installs Python 3.11
✅ Runs `pip install -r requirements.txt` from backend/
✅ Starts app with command from `Procfile`
✅ Sets `PORT` environment variable
✅ Provides HTTPS domain automatically
✅ Redeploys on git push
✅ Shows logs in real-time
✅ Auto-restarts on failure (configured for 10 retries)

Your deployment should work automatically! 🚀
