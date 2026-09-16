import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import svgr from 'vite-plugin-svgr';
import { VitePWA } from 'vite-plugin-pwa';
import { viteStaticCopy } from 'vite-plugin-static-copy';

export default defineConfig(({ mode }) => ({
  server: {
    port: 3000,
    proxy: {
      '/api/socket': 'ws://localhost:4000',
      '/api': 'http://localhost:4000',
    },
  },
  define: {
    // Some deps (e.g. react-draggable, used by react-rnd) read
    // process.env.NODE_ENV directly; Vite doesn't polyfill `process` in the
    // browser, so without this it throws ReferenceError: process is not defined.
    'process.env.NODE_ENV': JSON.stringify(mode),
  },
  build: {
    outDir: 'build',
    target: 'es2022',
    rollupOptions: {
      output: {
        manualChunks(id) {
          if (/node_modules\/(react|react-dom|react-redux|@reduxjs)\//.test(id)) {
            return 'vendor-react';
          }
          if (/node_modules\/(@mui|@emotion|tss-react)\//.test(id)) {
            return 'vendor-mui';
          }
          if (/node_modules\/(maplibre-gl|@maplibre|@mapbox\/mapbox-gl-draw|@mapbox\/mapbox-gl-rtl-text)\//.test(id)) {
            return 'vendor-map';
          }
        },
      },
    },
  },
  esbuild: {
    target: 'es2022',
  },
  optimizeDeps: {
    esbuildOptions: {
      target: 'es2022',
    },
  },
  plugins: [
    svgr(),
    react(),
    VitePWA({
      includeAssets: ['favicon.ico'],
      workbox: {
        navigateFallbackDenylist: [/^\/api/],
        maximumFileSizeToCacheInBytes: 10 * 1024 * 1024,
        globPatterns: ['**/*.{js,css,html,woff,woff2,mp3}'],
      },
      manifest: {
        name: 'Drone Control Panel',
        short_name: 'DCP',
        description: 'A control panel for managing drone operations',
        theme_color: '#ffffff',
        icons: [
          {
            src: 'logo192.png',
            sizes: '192x192',
            type: 'image/png',
          },
          {
            src: 'logo512.png',
            sizes: '512x512',
            type: 'image/png',
          },
        ],
      },
    }),
    viteStaticCopy({
      targets: [{ src: 'node_modules/@mapbox/mapbox-gl-rtl-text/dist/mapbox-gl-rtl-text.js', dest: '' }],
    }),
  ],
}));
