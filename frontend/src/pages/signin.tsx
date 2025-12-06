import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import SigninForm from '../components/Auth/SigninForm';

export default function SigninPage() {
  useEffect(() => {
    // If user is already logged in, redirect to docs
    const token = localStorage.getItem('auth_token');
    if (token) {
      window.location.href = '/docs/intro';
    }
  }, []);

  return (
    <Layout
      title="Sign In - Physical AI Textbook"
      description="Sign in to access your personalized learning path"
    >
      <div className="container" style={{ marginTop: '2rem', marginBottom: '4rem', maxWidth: '600px' }}>
        <SigninForm />
      </div>
    </Layout>
  );
}
