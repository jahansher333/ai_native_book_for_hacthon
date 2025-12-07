@echo off
echo ========================================
echo   AI Robotics Textbook - Vercel Deploy
echo ========================================
echo.

echo Installing Vercel CLI...
call npm install -g vercel
echo.

echo Logging in to Vercel...
call vercel login
echo.

echo Deploying to Vercel...
cd frontend
call vercel --prod
echo.

echo ========================================
echo   Deployment Complete!
echo ========================================
echo.
echo Your site should be live at:
echo https://your-project.vercel.app
echo.
pause
