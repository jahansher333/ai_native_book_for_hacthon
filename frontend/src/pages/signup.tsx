import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/Auth/SignupForm';

export default function SignupPage() {
  useEffect(() => {
    // If user is already logged in, redirect to docs
    const token = localStorage.getItem('auth_token');
    if (token) {
      window.location.href = '/docs/intro';
    }
  }, []);

  return (
    <Layout
      title="Sign Up - Physical AI Textbook"
      description="Create your account to get personalized learning recommendations based on your hardware"
    >
      <div className="container" style={{ marginTop: '2rem', marginBottom: '4rem', maxWidth: '600px' }}>
        <SignupForm />
      </div>
    </Layout>
  );
}
