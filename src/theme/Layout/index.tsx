import React, { type ReactNode } from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import { useLocation } from '@docusaurus/router';
import type {WrapperProps} from '@docusaurus/types';

import ChatWidget from '@site/src/components/ChatWidget'; // Import the ChatWidget

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  // Determine if the current page is the homepage based on pathname
  const { pathname } = useLocation();
  const isHomepage = pathname === '/'; // Assuming homepage is at root

  return (
    <>
      <Layout {...props} />
      {/* Floating Chat Widget */}
      {/* Only show the chat widget on non-homepage pages */}
      {!isHomepage && (
        <ChatWidget />
      )}
    </>
  );
}

