# Production Deployment Checklist

Complete checklist for deploying Physical AI & Robotics platform to production.

## Pre-Deployment

### Code Quality
- [ ] All tests passing locally
- [ ] No console.log statements in production code
- [ ] No hardcoded secrets or API keys
- [ ] Environment variables properly configured
- [ ] TypeScript type errors resolved
- [ ] Python type hints added
- [ ] Code formatted (Black, Prettier)
- [ ] Linting passing (flake8, ESLint)

### Security
- [ ] All secrets moved to environment variables
- [ ] JWT_SECRET generated with `openssl rand -hex 32`
- [ ] CORS origins configured correctly
- [ ] SQL injection prevention verified
- [ ] XSS prevention with DOMPurify
- [ ] Rate limiting configured
- [ ] HTTPS enabled
- [ ] Security headers configured
- [ ] No sensitive data in logs
- [ ] API keys have minimum required permissions

### Backend
- [ ] `requirements.txt` up to date
- [ ] Database migrations tested
- [ ] Health check endpoint working
- [ ] API documentation generated
- [ ] Error handling comprehensive
- [ ] Logging configured properly
- [ ] Database connection pooling configured
- [ ] Timeout settings appropriate
- [ ] Memory limits set

### Frontend
- [ ] `package.json` dependencies locked
- [ ] Build process tested
- [ ] Docusaurus config updated with production URL
- [ ] Meta tags configured (SEO)
- [ ] Favicon added
- [ ] 404 page customized
- [ ] Loading states implemented
- [ ] Error boundaries added
- [ ] Analytics configured (optional)

## GitHub Setup

### Repository
- [ ] Repository created on GitHub
- [ ] All code pushed to main branch
- [ ] `.gitignore` configured properly
- [ ] README.md updated with deployment info
- [ ] LICENSE file added
- [ ] Branch protection rules enabled

### GitHub Secrets (Settings → Secrets → Actions)

**Required:**
- [ ] `GEMINI_API_KEY` - Google Gemini API key
- [ ] `GROQ_API_KEY` - Groq API key for LiteLLM
- [ ] `JWT_SECRET` - Generated secret for JWT tokens
- [ ] `DOCKER_USERNAME` - Docker Hub username
- [ ] `DOCKER_PASSWORD` - Docker Hub password/token

**Deployment Platform (choose one):**
- [ ] Railway: `RAILWAY_TOKEN`, `RAILWAY_PROJECT_ID`
- [ ] Render: `RENDER_API_KEY`, `RENDER_SERVICE_ID`
- [ ] AWS: `AWS_ACCESS_KEY_ID`, `AWS_SECRET_ACCESS_KEY`
- [ ] Vercel: `VERCEL_TOKEN`, `VERCEL_ORG_ID`, `VERCEL_PROJECT_ID`

### GitHub Actions
- [ ] Workflows enabled in repository
- [ ] Backend workflow tested
- [ ] Frontend workflow tested
- [ ] Deploy on push to main configured
- [ ] PR checks configured

### GitHub Pages
- [ ] Pages enabled in repository settings
- [ ] Source set to "GitHub Actions"
- [ ] Custom domain configured (optional)

## Deployment Platform Setup

### Backend (Choose One)

#### Railway
- [ ] Account created at railway.app
- [ ] Project created
- [ ] GitHub repository connected
- [ ] Environment variables added
- [ ] Postgres database provisioned
- [ ] Custom domain configured (optional)

#### Render
- [ ] Account created at render.com
- [ ] Web Service created
- [ ] GitHub repository connected
- [ ] Build command: `pip install -r requirements.txt`
- [ ] Start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- [ ] Environment variables added
- [ ] Postgres database created
- [ ] Auto-deploy enabled

#### AWS ECS/Fargate
- [ ] AWS account created
- [ ] ECR repository created
- [ ] ECS cluster created
- [ ] Task definition created
- [ ] Service created
- [ ] Load balancer configured
- [ ] IAM roles configured
- [ ] CloudWatch logs enabled

### Frontend (Choose One)

#### GitHub Pages (Default)
- [ ] Repository settings configured
- [ ] `docusaurus.config.ts` updated with correct URL
- [ ] Base URL set to `/repository-name/`
- [ ] Build tested locally

#### Vercel
- [ ] Account created at vercel.com
- [ ] Project imported from GitHub
- [ ] Build settings configured
- [ ] Environment variables added (if needed)
- [ ] Custom domain configured (optional)

#### Netlify
- [ ] Account created at netlify.com
- [ ] Site created from GitHub
- [ ] Build command: `npm run build`
- [ ] Publish directory: `build`
- [ ] Environment variables added
- [ ] Custom domain configured (optional)

## External Services

### Database (PostgreSQL)
- [ ] Neon account created (or other provider)
- [ ] Database created
- [ ] Connection string added to environment variables
- [ ] Migrations run
- [ ] Backups configured
- [ ] Connection pooling configured

### Vector Database (Qdrant)
- [ ] Qdrant Cloud account created (or local)
- [ ] Cluster created
- [ ] API key generated
- [ ] Collection created
- [ ] Connection tested
- [ ] Backup strategy defined

### API Keys
- [ ] Gemini API key obtained from Google AI Studio
- [ ] Groq API key obtained from Groq
- [ ] Rate limits understood
- [ ] Usage monitoring configured
- [ ] Billing alerts set up

## DNS & Domain (Optional)

- [ ] Custom domain purchased
- [ ] DNS records configured:
  - [ ] A record for backend (or CNAME)
  - [ ] CNAME for frontend
- [ ] SSL certificate provisioned
- [ ] Domain propagation verified
- [ ] Redirect HTTP → HTTPS enabled

## Monitoring & Logging

### Backend
- [ ] Health check endpoint accessible
- [ ] Logs configured (stdout/stderr)
- [ ] Error tracking setup (Sentry, optional)
- [ ] Uptime monitoring (UptimeRobot, optional)
- [ ] Performance monitoring (optional)

### Frontend
- [ ] Error tracking setup (optional)
- [ ] Analytics configured (Google Analytics, optional)
- [ ] Performance monitoring (Lighthouse)
- [ ] User feedback mechanism (optional)

### Alerts
- [ ] Deployment failure notifications
- [ ] Error rate alerts
- [ ] Uptime alerts
- [ ] API quota warnings

## Testing in Production

### Backend API
- [ ] Health check: `GET /api/health`
- [ ] Authentication: `POST /api/auth/signup`
- [ ] Login: `POST /api/auth/signin`
- [ ] RAG Query: `POST /api/query/chat`
- [ ] Personalization: `POST /api/personalize/chapter`
- [ ] Urdu Translation: `POST /api/translate/chapter`

### Frontend
- [ ] Homepage loads
- [ ] Navigation works
- [ ] Sign up flow works
- [ ] Sign in flow works
- [ ] RAG chat works
- [ ] Personalization works
- [ ] Urdu translation works
- [ ] Mobile responsiveness verified
- [ ] Dark mode works

### Performance
- [ ] Backend response time < 2s
- [ ] Frontend load time < 3s
- [ ] Lighthouse score > 80
- [ ] No memory leaks
- [ ] No 5xx errors

## Post-Deployment

### Documentation
- [ ] API documentation published
- [ ] Deployment guide updated
- [ ] README updated with live URLs
- [ ] Contributing guidelines added
- [ ] Changelog started

### Communication
- [ ] Deployment announced to team
- [ ] Users notified of new features
- [ ] Known issues documented
- [ ] Support channels established

### Maintenance
- [ ] Backup strategy implemented
- [ ] Update schedule defined
- [ ] Rollback procedure documented
- [ ] Incident response plan created

## Security Hardening

- [ ] Rate limiting enabled
- [ ] CORS properly configured
- [ ] Input validation on all endpoints
- [ ] SQL parameterized queries
- [ ] No eval() or similar dangerous functions
- [ ] Dependencies scanned for vulnerabilities
- [ ] Security headers configured:
  - [ ] X-Content-Type-Options: nosniff
  - [ ] X-Frame-Options: DENY
  - [ ] X-XSS-Protection: 1; mode=block
  - [ ] Content-Security-Policy configured
  - [ ] Strict-Transport-Security enabled

## Compliance (if applicable)

- [ ] Privacy policy added
- [ ] Terms of service added
- [ ] Cookie consent implemented
- [ ] GDPR compliance verified
- [ ] Data retention policy defined
- [ ] User data export capability

## Performance Optimization

### Backend
- [ ] Database indexes created
- [ ] Query optimization done
- [ ] Caching implemented (Redis, optional)
- [ ] CDN configured for static files
- [ ] Gzip compression enabled
- [ ] Connection pooling configured

### Frontend
- [ ] Images optimized
- [ ] Code splitting implemented
- [ ] Lazy loading configured
- [ ] Bundle size optimized
- [ ] CDN configured
- [ ] Service worker configured (optional)

## Cost Optimization

- [ ] Free tier limits understood
- [ ] Auto-scaling configured appropriately
- [ ] Unused resources identified
- [ ] Budget alerts configured
- [ ] Resource utilization monitored
- [ ] Cost estimates documented

## Final Verification

### Smoke Tests
```bash
# Backend health
curl https://your-backend-url.com/api/health

# Frontend homepage
curl -I https://your-frontend-url.com

# API test
curl -X POST https://your-backend-url.com/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!"}'
```

### Load Testing (Optional)
```bash
# Install Apache Bench
apt-get install apache2-utils

# Test backend
ab -n 1000 -c 10 https://your-backend-url.com/api/health

# Expected: < 2s average response time
```

### Security Scan (Optional)
```bash
# Scan Docker image
docker scan your-dockerhub-username/ai-robotics-backend:latest

# Scan dependencies
pip install safety
safety check

npm audit
```

## Rollback Plan

If deployment fails:

1. **GitHub:**
   ```bash
   git revert HEAD
   git push origin main
   ```

2. **Railway:**
   ```bash
   railway rollback
   ```

3. **Render:**
   - Dashboard → Deployments → Previous → Redeploy

4. **Docker Hub:**
   - Revert to previous image tag

## Success Criteria

✅ All services running without errors
✅ Health checks passing
✅ Frontend accessible and functional
✅ Backend API responding correctly
✅ Authentication working
✅ Personalization working
✅ Urdu translation working
✅ No critical security vulnerabilities
✅ Response times acceptable
✅ SSL certificates valid
✅ Monitoring configured
✅ Logs accessible

## Post-Launch Monitoring (First 24 Hours)

- [ ] Monitor error rates
- [ ] Check API quotas
- [ ] Verify uptime
- [ ] Monitor response times
- [ ] Check user feedback
- [ ] Review logs for issues
- [ ] Verify backups running

## Long-Term Maintenance

### Weekly
- [ ] Review error logs
- [ ] Check API usage
- [ ] Monitor costs
- [ ] Review security alerts

### Monthly
- [ ] Update dependencies
- [ ] Review performance metrics
- [ ] Optimize database
- [ ] Review and rotate secrets
- [ ] Check backup integrity

### Quarterly
- [ ] Security audit
- [ ] Performance review
- [ ] Cost optimization review
- [ ] Dependency major updates
- [ ] Load testing

## Notes

- Keep this checklist updated as you deploy
- Use issues/tickets to track incomplete items
- Document any deviations from the plan
- Share learnings with the team

---

**Last Updated:** 2025-12-07
**Deployment Environment:** Production
**Estimated Time:** 2-4 hours for first deployment
