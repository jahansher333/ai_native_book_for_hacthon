import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';
import {useLocation} from '@docusaurus/router';
import ChapterWrapper from '../../../components/Personalize/ChapterWrapper';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  const location = useLocation();

  // Extract chapter ID from pathname
  // Example: /docs/00-intro/intro → 00-intro/intro
  const chapterId = location.pathname
    .replace('/docs/', '')
    .replace(/\/$/, '')
    .replace(/^\//, '');

  return (
    <ChapterWrapper chapterId={chapterId}>
      <Layout {...props} />
    </ChapterWrapper>
  );
}
