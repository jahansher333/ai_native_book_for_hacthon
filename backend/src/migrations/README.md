# Database Migrations

This directory contains SQL migration scripts for the Physical AI Textbook backend.

## Migration Files

### 001_add_profile.sql
**Date**: 2025-12-06
**Feature**: 004-better-auth-profile
**Purpose**: Add profile-based authentication with hardware/experience tracking

Creates:
- `users` table with profile JSONB column
- `sessions` table for JWT token management
- Indexes for performance (email B-tree, profile GIN)

Profile structure:
```json
{
  "hasRTX": true,
  "hasJetson": false,
  "hasRobot": false,
  "experience": "beginner",
  "profile_version": 1,
  "created_at": "2025-12-06T..."
}
```

## Running Migrations

### Method 1: Automatic (on server startup)
Migrations run automatically when the FastAPI backend starts. Tables are created using SQLAlchemy `Base.metadata.create_all()`.

### Method 2: Manual (using psql)
```bash
# Connect to Neon Postgres
psql $NEON_DATABASE_URL

# Run migration
\i backend/src/migrations/001_add_profile.sql

# Verify tables
\dt

# Verify profile column
\d users

# Exit
\q
```

### Method 3: Manual (using Python)
```python
from backend.src.services.auth_service import engine
from backend.src.models.user import Base

# Create all tables
Base.metadata.create_all(engine)
print("Tables created successfully")
```

## Verification Queries

### Check if users table exists
```sql
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public' AND table_name = 'users';
```

### Check profile column structure
```sql
SELECT column_name, data_type
FROM information_schema.columns
WHERE table_name = 'users' AND column_name = 'profile';
```

### Example profile queries
```sql
-- Find users with RTX GPU
SELECT email, profile->>'hasRTX' as has_rtx
FROM users
WHERE profile->>'hasRTX' = 'true';

-- Find advanced users with Jetson
SELECT email, profile->>'experience' as experience
FROM users
WHERE profile->>'experience' = 'advanced'
  AND profile->>'hasJetson' = 'true';

-- Count users by experience level
SELECT profile->>'experience' as level, COUNT(*)
FROM users
GROUP BY profile->>'experience';

-- Count users by hardware
SELECT
  COUNT(*) FILTER (WHERE profile->>'hasRTX' = 'true') as rtx_users,
  COUNT(*) FILTER (WHERE profile->>'hasJetson' = 'true') as jetson_users,
  COUNT(*) FILTER (WHERE profile->>'hasRobot' = 'true') as robot_users
FROM users;
```

## Rollback (if needed)

To remove auth tables:
```sql
DROP TABLE IF EXISTS sessions CASCADE;
DROP TABLE IF EXISTS users CASCADE;
```

**WARNING**: This will delete all user accounts and sessions. Use with caution!

## Database Connection

All migrations use the existing Neon Postgres database:
- Environment variable: `NEON_DATABASE_URL`
- Format: `postgresql://user:password@your-neon-endpoint/dbname`
- Shared with RAG chatbot backend (single source of truth)

## Notes

- All migrations are **idempotent** (safe to run multiple times)
- Uses `CREATE TABLE IF NOT EXISTS`, `ALTER TABLE ADD COLUMN IF NOT EXISTS`
- Profile JSONB column supports flexible schema evolution
- GIN index on profile enables fast JSON queries
- Password hashes stored using bcrypt (cost factor 12)
- JWT tokens stored in `sessions` table with expiration timestamps
