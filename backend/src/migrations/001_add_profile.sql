-- Migration: Add profile column to users table
-- Date: 2025-12-06
-- Feature: 004-better-auth-profile
-- Purpose: Store user hardware and experience profile for personalized content

-- Create users table if not exists (Better-Auth equivalent structure)
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    email_verified BOOLEAN DEFAULT FALSE,
    profile JSONB DEFAULT '{}'::JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Add profile column if users table already exists but lacks it
ALTER TABLE users ADD COLUMN IF NOT EXISTS profile JSONB DEFAULT '{}'::JSONB;

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
CREATE INDEX IF NOT EXISTS idx_users_profile ON users USING GIN (profile);

-- Create sessions table for JWT token management
CREATE TABLE IF NOT EXISTS sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token TEXT NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_sessions_token ON sessions(token);
CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);

-- Insert sample profile structure for documentation
COMMENT ON COLUMN users.profile IS 'JSONB structure: {"hasRTX": boolean, "hasJetson": boolean, "hasRobot": boolean, "experience": "beginner"|"intermediate"|"advanced", "profile_version": 1}';

-- Example profile queries for reference:
-- Find users with RTX GPU: SELECT * FROM users WHERE profile->>'hasRTX' = 'true';
-- Find advanced users with Jetson: SELECT * FROM users WHERE profile->>'experience' = 'advanced' AND profile->>'hasJetson' = 'true';
-- Count users by experience level: SELECT profile->>'experience' as level, COUNT(*) FROM users GROUP BY profile->>'experience';
