import js from '@eslint/js';
import { configs as importConfigs } from 'eslint-plugin-import-x';
import prettierRecommended from 'eslint-plugin-prettier/recommended';
import globals from 'globals';

export default [
  {
    ignores: ['node_modules/**', 'logs/**', 'data/**', 'fbmsglib/dist/**'],
  },
  js.configs.recommended,
  importConfigs['flat/recommended'],
  {
    files: ['**/*.js'],
    languageOptions: {
      ecmaVersion: 'latest',
      sourceType: 'module',
      globals: {
        ...globals.node,
        ...globals.es2021,
      },
    },
    settings: {
      'import-x/resolver': {
        node: {
          extensions: ['.js', '.json'],
        },
      },
    },
    rules: {
      'no-prototype-builtins': 'off',
      'no-empty': ['error', { allowEmptyCatch: true }],
      'no-unused-vars': ['warn', { argsIgnorePattern: '^_', varsIgnorePattern: '^_' }],
      'import-x/no-unresolved': [
        'warn',
        {
          ignore: ['^fbmsglib'],
        },
      ],
    },
  },
  prettierRecommended,
];
