import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import svgr from 'vite-plugin-svgr';
import { VitePWA } from 'vite-plugin-pwa';

export default defineConfig(() => ({
  server: {
    port: 3000,
    proxy: {
      '/api/socket': 'ws://localhost:4000',
      '/api': 'http://localhost:4000',
    },
  },
  build: {
    outDir: 'build',
  },
  plugins: [svgr(), react(), VitePWA()],
}));
