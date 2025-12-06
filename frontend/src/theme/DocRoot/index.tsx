import React, { JSX } from 'react';
import DocRoot from '@theme-original/DocRoot';
import type DocRootType from '@theme/DocRoot';
import type {WrapperProps} from '@docusaurus/types';
import AuthGuard from '../../components/Auth/AuthGuard';

type Props = WrapperProps<typeof DocRootType>;

export default function DocRootWrapper(props: Props): JSX.Element {
  return (
    <AuthGuard>
      <DocRoot {...props} />
    </AuthGuard>
  );
}
