# Unified ChapterWrapper - Testing Guide

## Overview
This document provides step-by-step testing instructions for the unified ChapterWrapper component that handles both **Personalization** and **Urdu Translation** features.

## Features Implemented

### 1. Unified Three-Button Layout
- **Button 1**: "✨ Personalize for Me" (only visible when logged in)
- **Button 2**: "اردو میں پڑھیں" / "English میں واپس" (toggle)
- **Button 3**: "🔄 Restore Original" (only visible when content is modified)

### 2. Comprehensive Status Badges
- **Personalized**: `✨ Personalized for Advanced + Jetson Owner + RTX GPU (Cached)`
- **Urdu**: `اردو میں دکھایا جا رہا ہے (Cached) 🇵🇰`

### 3. Backend Integration
- **Personalization**: Uses LiteLLM with `groq/llama-3.3-70b-versatile`
- **Urdu Translation**: Uses LiteLLM with `groq/mixtral-8x7b-32768`

## Test Scenarios

### Test 1: View Original Content (Not Logged In)
**Steps:**
1. Navigate to any chapter without logging in
2. Verify only the Urdu button is visible
3. Click "اردو میں پڑھیں"
4. Verify Urdu content displays with RTL text
5. Verify badge shows: `اردو میں دکھایا جا رہا ہے 🇵🇰`
6. Click "English میں واپس"
7. Verify original English content is restored

**Expected Result:**
- Personalize button NOT visible (user not authenticated)
- Urdu translation works without login
- Content switches seamlessly between English and Urdu

---

### Test 2: Personalize Content (Logged In)
**Steps:**
1. Log in with a user account
2. Set user profile (e.g., Advanced, hasJetson: true, hasRTX: true)
3. Navigate to any chapter
4. Click "✨ Personalize for Me"
5. Wait for personalization to complete (loading spinner)
6. Verify personalized content displays
7. Verify badge shows: `✨ Personalized for Advanced + Jetson Owner + RTX GPU`

**Expected Result:**
- Content is rewritten based on user profile
- Badge accurately reflects user's experience + hardware
- Jetson-specific tips appear for Jetson owners
- Advanced concepts explained without basics

---

### Test 3: Urdu Translation After Personalization
**Steps:**
1. Complete Test 2 (personalize content)
2. Click "اردو میں پڑھیں" button
3. Verify Urdu translation of ORIGINAL content (not personalized)
4. Verify badge shows: `اردو میں دکھایا جا رہا ہے 🇵🇰`
5. Click "🔄 Restore Original"
6. Verify original English content is restored

**Expected Result:**
- Urdu translation works independently of personalization
- Code blocks remain in English
- Technical terms properly transliterated (e.g., "ROS 2" → "روبوٹک آپریٹنگ سسٹم 2")
- Prices preserved ($249 stays as $249, not converted to PKR)

---

### Test 4: Caching Behavior
**Steps:**
1. Personalize a chapter (first time)
2. Note badge does NOT show "(Cached)"
3. Restore original
4. Personalize same chapter again
5. Verify badge shows "(Cached)"
6. Repeat for Urdu translation

**Expected Result:**
- First request takes 5-15 seconds
- Cached requests return instantly (<100ms)
- Badge indicates cached vs fresh generation
- Cache persists for 7 days (as per localStorage TTL)

---

### Test 5: Error Handling
**Steps:**
1. Disconnect backend server (stop FastAPI)
2. Try to personalize
3. Verify error message appears
4. Try Urdu translation
5. Verify error message appears
6. Restart backend
7. Verify features work again

**Expected Result:**
- User-friendly error messages
- No crashes or blank screens
- Buttons remain functional after errors
- Errors displayed in both English and Urdu (for Urdu feature)

---

### Test 6: Loading States
**Steps:**
1. Click "✨ Personalize for Me"
2. Verify button shows spinner + "Personalizing..."
3. Verify other buttons are disabled during loading
4. Wait for completion
5. Repeat for Urdu translation

**Expected Result:**
- Spinner animation visible during loading
- Button text changes to "Personalizing..." / "Translating..."
- Other buttons disabled to prevent conflicts
- Loading state clears after completion or error

---

### Test 7: Mobile Responsiveness
**Steps:**
1. Open chapter on mobile device or resize browser to <768px
2. Verify buttons stack vertically
3. Verify all buttons are full-width
4. Test all features work on mobile

**Expected Result:**
- Buttons stack in column layout
- Full-width buttons for easy tapping
- All features fully functional on mobile
- Text remains readable

---

### Test 8: Dark Mode Support
**Steps:**
1. Enable dark mode in browser/OS
2. Navigate to any chapter
3. Verify button colors are readable
4. Personalize content
5. Verify personalized content background is visible
6. Switch to Urdu
7. Verify Urdu content background is visible

**Expected Result:**
- All buttons have sufficient contrast in dark mode
- Badge colors work in dark mode
- Content backgrounds visible but not overwhelming
- Error messages readable in dark mode

---

## Backend API Verification

### Personalize API
**Endpoint**: `POST http://localhost:8001/api/personalize/chapter`

**Request Body**:
```json
{
  "chapterId": "ros2/introduction",
  "originalContent": "ROS 2 is a middleware...",
  "userProfile": {
    "experience": "advanced",
    "hasRTX": true,
    "hasJetson": true,
    "hasRobot": false
  }
}
```

**Expected Response**:
```json
{
  "success": true,
  "personalizedContent": "<personalized HTML content>",
  "metadata": {
    "generationTime": 0,
    "cached": false
  }
}
```

**Auth Required**: Yes (Bearer token in Authorization header)

---

### Urdu Translation API
**Endpoint**: `POST http://localhost:8001/api/translate/chapter`

**Request Body**:
```json
{
  "chapterId": "ros2/introduction",
  "originalContent": "ROS 2 is a middleware for robotics..."
}
```

**Expected Response**:
```json
{
  "chapterId": "ros2/introduction",
  "translatedContent": "<Urdu HTML content>",
  "timestamp": 1234567890,
  "cached": false
}
```

**Auth Required**: No (public endpoint)

---

## Troubleshooting

### Issue: Personalize button not showing
**Solution**: Verify user is logged in and profile is set in UserProfileContext

### Issue: Urdu text not displaying properly
**Solution**: Verify Noto Nastaliq Urdu font is loaded in docusaurus.config.ts (line 58-60)

### Issue: Backend returns 500 error
**Solution**: Check GROQ_API_KEY is set in backend/.env

### Issue: Translation takes too long
**Solution**: Shorten chapter content (max 30k tokens ≈ 120k characters)

### Issue: Code blocks showing in Urdu
**Solution**: Verify CSS rule `.content.urduContent pre, code { direction: ltr; }`

### Issue: Prices converted to PKR
**Solution**: Check Urdu translator prompt preserves prices as-is

---

## Success Criteria

✅ All three buttons render correctly
✅ Personalization works for logged-in users
✅ Urdu translation works for all users (no auth required)
✅ Badges accurately reflect active state
✅ Caching reduces load time to <100ms
✅ Error messages are user-friendly
✅ Mobile layout works properly
✅ Dark mode colors are readable
✅ Code blocks remain in English in Urdu mode
✅ Prices preserved exactly ($249 not converted)

---

## Next Steps After Testing

1. **Performance Optimization**: Add server-side caching to reduce redundant API calls
2. **Analytics**: Track personalization usage per user profile type
3. **A/B Testing**: Test different badge wording for better UX
4. **Internationalization**: Add support for more languages (Arabic, French, Spanish)
5. **Advanced Personalization**: Add project-based personalization (sim vs real robot)
