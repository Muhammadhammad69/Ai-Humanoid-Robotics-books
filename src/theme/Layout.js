import React from 'react';
import Layout from '@theme-original/Layout';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

// Add the ChatWidget to the main layout so it appears on all pages
export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
        {/* ChatWidget will be rendered on all pages */}
        <ChatWidget displayMode="floating" />
      </Layout>
    </>
  );
}