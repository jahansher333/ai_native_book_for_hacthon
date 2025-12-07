# Vercel Deployment Guide

## Quick Deploy to Vercel

### Method 1: Via Vercel Dashboard (Easiest)

1. **Go to Vercel**: https://vercel.com

2. **Import Project**:
   - Click "Add New" → "Project"
   - Import from GitHub: `jahansher333/Ai_Native_Books_Pyhsical_Ai`
   - Select the repository

3. **Configure Project**:
   ```
   Framework Preset: Other
   Root Directory: frontend
   Build Command: npm run build
   Output Directory: build
   Install Command: npm install
   ```

4. **Environment Variables** (if needed):
   ```
   NODE_ENV=production
   ```

5. **Deploy**:
   - Click "Deploy"
   - Wait 2-3 minutes
   - Your site will be live at: `https://your-project.vercel.app`

### Method 2: Via Vercel CLI

```bash
# Install Vercel CLI
npm install -g vercel

# Login to Vercel
vercel login

# Deploy from project root
cd "D:\New folder (6)\ai_robotics_book"
vercel

# Follow prompts:
# - Set up and deploy? Yes
# - Which scope? Your account
# - Link to existing project? No
# - Project name: ai-robotics-textbook
# - Directory: ./frontend
# - Override settings? No

# Deploy to production
vercel --prod
```

### Method 3: Via GitHub Actions (Automated)

The workflow is already created in `.github/workflows/frontend-deploy.yml`

**Add Vercel Secrets to GitHub**:
1. Go to repository Settings → Secrets → Actions
2. Add:
   ```
   VERCEL_TOKEN=your_vercel_token
   VERCEL_ORG_ID=your_org_id
   VERCEL_PROJECT_ID=your_project_id
   ```

**Get Vercel Token**:
1. Go to https://vercel.com/account/tokens
2. Create new token
3. Copy and add to GitHub Secrets

**Get Org ID and Project ID**:
```bash
# Install Vercel CLI
npm install -g vercel

# Link project
cd frontend
vercel link

# View project settings
cat .vercel/project.json
# This will show your orgId and projectId
```

Then push to main branch:
```bash
git push origin main
```

## Vercel Configuration Files

### Root vercel.json
Location: `./vercel.json`

```json
{
  "version": 2,
  "name": "ai-robotics-textbook",
  "buildCommand": "cd frontend && npm run build",
  "outputDirectory": "frontend/build",
  "installCommand": "cd frontend && npm install",
  "framework": "docusaurus"
}
```

### Frontend vercel.json
Location: `./frontend/vercel.json`

```json
{
  "version": 2,
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus"
}
```

## Update Docusaurus Config for Vercel

Edit `frontend/docusaurus.config.ts`:

```typescript
const config: Config = {
  url: 'https://your-project.vercel.app', // Update with your Vercel URL
  baseUrl: '/', // Vercel uses root path

  // ... rest of config
};
```

## Custom Domain (Optional)

### Add Custom Domain in Vercel:

1. Go to Project Settings → Domains
2. Add your domain: `example.com`
3. Configure DNS:
   ```
   Type: A
   Name: @
   Value: 76.76.21.21

   Type: CNAME
   Name: www
   Value: cname.vercel-dns.com
   ```

### Update Docusaurus Config:
```typescript
url: 'https://your-custom-domain.com',
baseUrl: '/',
```

## Environment Variables

If you need environment variables:

**Via Vercel Dashboard**:
1. Project Settings → Environment Variables
2. Add:
   ```
   NODE_ENV=production
   NEXT_PUBLIC_API_URL=https://your-backend-url.com
   ```

**Via Vercel CLI**:
```bash
vercel env add NODE_ENV production
vercel env add NEXT_PUBLIC_API_URL https://your-backend-url.com
```

## Troubleshooting

### Build Fails with "command not found"

**Solution**: Ensure `package.json` has correct scripts:
```json
{
  "scripts": {
    "build": "docusaurus build",
    "start": "docusaurus start",
    "serve": "docusaurus serve"
  }
}
```

### 404 on Routes

**Solution**: Ensure `vercel.json` has correct rewrites:
```json
{
  "rewrites": [
    { "source": "/:path*", "destination": "/:path*" }
  ]
}
```

### Assets Not Loading

**Solution**: Update `docusaurus.config.ts`:
```typescript
url: 'https://your-project.vercel.app',
baseUrl: '/',  // Not '/ai_robotics_book/'
```

### Build Timeout

**Solution**:
1. Check build logs in Vercel dashboard
2. Ensure dependencies are in `package.json`, not `devDependencies`
3. Increase build timeout in Vercel settings (Pro plan)

## Deployment Preview

Vercel automatically creates preview deployments for:
- Every push to any branch
- Every pull request

Preview URL format: `https://ai-robotics-textbook-git-branch-name-username.vercel.app`

## Monitoring

### View Deployments:
```bash
vercel ls
```

### View Logs:
```bash
vercel logs your-deployment-url
```

### Rollback:
1. Go to Vercel Dashboard → Deployments
2. Find previous successful deployment
3. Click "..." → "Promote to Production"

Or via CLI:
```bash
vercel rollback
```

## Performance Optimization

### Enable Edge Network:
Vercel automatically uses its global CDN

### Enable Compression:
Add to `vercel.json`:
```json
{
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "Content-Encoding",
          "value": "gzip"
        }
      ]
    }
  ]
}
```

### Optimize Images:
Vercel automatically optimizes images

## Cost

- **Hobby (Free)**:
  - 100 GB bandwidth/month
  - 100 deployments/day
  - Perfect for personal projects

- **Pro ($20/month)**:
  - 1 TB bandwidth/month
  - Unlimited deployments
  - Custom domains
  - Team collaboration

## Comparison: Vercel vs GitHub Pages

| Feature | Vercel | GitHub Pages |
|---------|--------|--------------|
| Build Time | ~2-3 min | ~5-10 min |
| Custom Domain | ✅ Free SSL | ✅ Free SSL |
| Preview Deployments | ✅ Yes | ❌ No |
| Edge Network | ✅ Global | ✅ GitHub CDN |
| Bandwidth | 100 GB/month | Unlimited |
| Build Minutes | Unlimited | 2000/month |
| **Best For** | Production apps | Open source docs |

## Connect Backend

Update frontend to use backend URL:

**Create `.env.local`** (frontend directory):
```env
NEXT_PUBLIC_BACKEND_URL=https://your-backend.railway.app
```

**Update API calls** (if needed):
```typescript
const BACKEND_URL = process.env.NEXT_PUBLIC_BACKEND_URL || 'http://localhost:8001';

fetch(`${BACKEND_URL}/api/personalize/chapter`, {
  // ...
});
```

## CI/CD Integration

Vercel automatically deploys when:
1. Push to `main` branch → Production
2. Push to any branch → Preview
3. Open PR → Preview with comment

**Disable auto-deploy** (optional):
```bash
# Add to vercel.json
{
  "git": {
    "deploymentEnabled": {
      "main": true,
      "preview": false
    }
  }
}
```

## Security Headers

Already configured in `frontend/vercel.json`:
- X-Content-Type-Options: nosniff
- X-Frame-Options: DENY
- X-XSS-Protection: 1; mode=block

## Analytics (Optional)

Enable Vercel Analytics:
1. Go to Project Settings → Analytics
2. Enable Vercel Analytics
3. Add to `frontend/package.json`:
   ```bash
   npm install @vercel/analytics
   ```

4. Add to `src/theme/Root.tsx`:
   ```typescript
   import { Analytics } from '@vercel/analytics/react';

   export default function Root({children}) {
     return (
       <>
         {children}
         <Analytics />
       </>
     );
   }
   ```

## Summary

### Quick Start Commands:
```bash
# Option 1: CLI
npm install -g vercel
vercel login
cd "D:\New folder (6)\ai_robotics_book\frontend"
vercel --prod

# Option 2: Dashboard
# Go to vercel.com → Import from GitHub

# Option 3: GitHub Actions
# Already configured, just add secrets
```

### After Deployment:
1. ✅ Your site is live at: `https://your-project.vercel.app`
2. ✅ Auto-deploys on git push
3. ✅ Preview deployments for PRs
4. ✅ Global CDN
5. ✅ Free SSL certificate

**Your Docusaurus site will be deployed in 2-3 minutes!** 🚀

## Next Steps

1. Deploy frontend to Vercel
2. Deploy backend to Railway/Render
3. Update frontend with backend URL
4. Test all features
5. Add custom domain (optional)
