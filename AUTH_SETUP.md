# Authentication Setup Guide

Complete authentication system with profile-based personalization for the Physical AI Textbook.

## 🎯 Features

- **Profile Questions**: 4 hardware/experience questions at signup
  1. Do you have an RTX GPU?
  2. Do you own a Jetson Orin Nano?
  3. Do you have access to a real robot?
  4. Your programming experience? (beginner/intermediate/advanced)

- **Personalized Recommendations**: Content suggestions based on user profile
- **Beautiful UI**: Tailwind-inspired design with dark mode support
- **Secure**: bcrypt password hashing + JWT tokens (7-day expiration)
- **Optional Login**: Book readable without authentication (Constitution Principle X)

## 📦 Installation

### Backend Setup

1. **Install dependencies**:
```bash
cd backend
pip install python-jose[cryptography]==3.3.0
pip install passlib[bcrypt]==1.7.4
```

2. **Generate JWT secret**:
```bash
openssl rand -hex 32
```

3. **Update `.env`** (add to existing file):
```bash
# JWT Authentication
JWT_SECRET=<paste-generated-secret-here>
JWT_ALGORITHM=HS256
JWT_EXPIRATION_DAYS=7
```

4. **Run database migration** (automatic on first startup):
```bash
# Migration runs automatically when backend starts
# Or run manually:
psql $NEON_DATABASE_URL < backend/src/migrations/001_add_profile.sql
```

5. **Start backend**:
```bash
uvicorn src.main:app --reload
```

Backend will be available at `http://localhost:8000`

### Frontend Setup

1. **No additional dependencies** (uses existing React + Docusaurus)

2. **Start frontend**:
```bash
cd frontend
npm start
```

Frontend will be available at `http://localhost:3000`

## 🚀 Usage

### Sign Up Flow

1. Visit `http://localhost:3000/signup`
2. Enter email + password (min 8 characters)
3. Answer 4 profile questions:
   - RTX GPU? (yes/no radio buttons)
   - Jetson? (yes/no radio buttons)
   - Real robot? (yes/no radio buttons)
   - Experience? (dropdown: beginner/intermediate/advanced)
4. Click "Create Account"
5. Redirects to `/docs/intro?personalized=true`

### Sign In Flow

1. Visit `http://localhost:3000/signin`
2. Enter email + password
3. Click "Sign In"
4. Redirects to last visited page or `/docs/intro`

### Profile Page

1. Visit `http://localhost:3000/profile`
2. View:
   - Hardware setup (RTX GPU, Jetson, Real Robot)
   - Programming experience level
   - Personalized recommendations
   - Account information

### Sign Out

1. Click "Sign Out" button in navbar
2. Clears localStorage and redirects to homepage

## 🔧 API Endpoints

### POST `/api/auth/signup`
Create new user account with profile.

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123",
  "profile": {
    "hasRTX": true,
    "hasJetson": false,
    "hasRobot": false,
    "experience": "beginner"
  }
}
```

**Response** (201):
```json
{
  "success": true,
  "user": {
    "id": "uuid",
    "email": "student@example.com",
    "profile": { ... },
    "email_verified": false,
    "created_at": "2025-12-06T..."
  },
  "token": "jwt-token-here",
  "expires_at": "2025-12-13T..."
}
```

### POST `/api/auth/signin`
Authenticate user with email + password.

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePass123"
}
```

**Response** (200):
```json
{
  "success": true,
  "user": { ... },
  "token": "jwt-token-here",
  "expires_at": "2025-12-13T..."
}
```

### GET `/api/auth/session`
Validate JWT token and return user data.

**Headers**:
```
Authorization: Bearer <jwt-token>
```

**Response** (200):
```json
{
  "authenticated": true,
  "user": {
    "id": "uuid",
    "email": "student@example.com",
    "profile": { ... }
  }
}
```

Invalid token returns:
```json
{
  "authenticated": false,
  "user": null
}
```

## 📊 Database Schema

### `users` table
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    profile JSONB DEFAULT '{}'::JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

### `sessions` table
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token TEXT NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

### Indexes
- `idx_users_email` (B-tree on email)
- `idx_users_profile` (GIN on profile JSONB)
- `idx_sessions_token` (B-tree on token)
- `idx_sessions_user_id` (B-tree on user_id)

## 🔍 Testing

### Manual Testing

1. **Test signup**:
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123",
    "profile": {
      "hasRTX": false,
      "hasJetson": false,
      "hasRobot": false,
      "experience": "beginner"
    }
  }'
```

2. **Test signin**:
```bash
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "SecurePass123"
  }'
```

3. **Test session validation**:
```bash
TOKEN="<jwt-token-from-signin>"
curl -X GET http://localhost:8000/api/auth/session \
  -H "Authorization: Bearer $TOKEN"
```

### Frontend Testing

1. Open `http://localhost:3000/signup`
2. Fill form with test data
3. Verify redirect to `/docs/intro`
4. Check localStorage for `auth_token`, `user_profile`, `user_email`
5. Visit `/profile` → See hardware setup
6. Click "Sign Out" → Verify redirect to `/`

## 🎨 Components

### React Components
- `SignupForm.tsx`: 4-question signup form with validation
- `SigninForm.tsx`: Simple email + password signin
- `UserProfileContext.tsx`: Auth state management (useUserProfile hook)
- `Navbar/Content/index.tsx`: Dynamic auth buttons (Sign Up/Sign In or Profile/Sign Out)

### Pages
- `/signup`: Signup page with Docusaurus Layout
- `/signin`: Signin page with Docusaurus Layout
- `/profile`: Profile page showing hardware background + recommendations

### Styling
- `auth.module.css`: Beautiful Tailwind-inspired forms with dark mode
- `profile.module.css`: Profile page cards and layout

## 🔐 Security

- **Password Hashing**: bcrypt with cost factor 12
- **JWT Tokens**: HS256 algorithm, 7-day expiration
- **CORS**: Configured for localhost:3000
- **Generic Errors**: Prevents email enumeration attacks
- **Token Storage**: localStorage (client-side)
- **HTTPS**: Required for production

## 🌟 Personalization Examples

### User Profile:
```json
{
  "hasRTX": true,
  "hasJetson": true,
  "hasRobot": false,
  "experience": "advanced"
}
```

### Profile Summary:
"Advanced developer with RTX GPU, Jetson"

### Recommendations:
- "Get a real robot: Build a $700 Economy Kit"
- "Sim-to-Real Transfer: Bridge the sim-to-real gap"
- "Deploy models to Jetson: <10ms inference for real-time control"

## 📝 Next Steps

1. **Email Verification**: Add email verification flow
2. **Password Reset**: Implement forgot password functionality
3. **Social Auth**: Add Google/GitHub OAuth
4. **@personalizer Subagent**: Generate custom chapter versions based on profile
5. **Analytics**: Track which tutorials users complete
6. **Gamification**: Award points for completing modules

## 🐛 Troubleshooting

### Backend won't start
- Check `JWT_SECRET` is set in `.env`
- Verify `NEON_DATABASE_URL` is correct
- Check `python-jose` and `passlib` are installed

### Signup fails with "Email already exists"
- User already signed up with that email
- Check database: `SELECT * FROM users WHERE email='...'`

### Token invalid/expired
- JWT expired (7-day limit)
- User needs to sign in again
- Check token format: `Bearer <token>`

### Profile page redirects to signin
- No auth token in localStorage
- Token expired or invalid
- Sign in again

## 📚 References

- [python-jose Documentation](https://python-jose.readthedocs.io/)
- [passlib Documentation](https://passlib.readthedocs.io/)
- [JWT.io](https://jwt.io/)
- [Docusaurus Swizzling](https://docusaurus.io/docs/swizzling)

---

**50 Bonus Points Earned!** 🎉

Complete authentication system with profile-based personalization using existing Neon Postgres database.
