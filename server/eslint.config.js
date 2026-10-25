import js from '@eslint/js';
import { configs as importConfigs } from 'eslint-plugin-import-x';
import esX from 'eslint-plugin-es-x';
import prettierRecommended from 'eslint-plugin-prettier/recommended';
import globals from 'globals';

export default [
  {
    ignores: ['node_modules/**', 'logs/**', 'data/**', 'fbmsglib/dist/**', 'utils/proccessThermalImg/.venv/**'],
  },
  js.configs.recommended,
  importConfigs['flat/recommended'],
  // Server runs on Node 20 (.nvmrc v20.19): flags ES2024+ builtins/syntax
  // (Map.groupBy, Promise.withResolvers, Set.prototype.union, ...) that
  // engines>=20 alone doesn't catch. See server/CLAUDE.md Gotcha #13.
  esX.configs['flat/restrict-to-es2023'],
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
