import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { categoryModel } from '../models/category.js';

// createCategory writes the git-tracked devices_msg.yaml. These tests only exercise the
// path that must NOT write (the category already exists), so they never touch the real file.

describe('categoryModel.createCategory', () => {
  test('refuses to overwrite an existing category and leaves it untouched', () => {
    const name = categoryModel.getAll()[0];
    const before = structuredClone(categoryModel.getCategory(name));

    const result = categoryModel.createCategory(name, { subscribers: {}, services: {} });

    assert.equal(result, null);
    assert.deepEqual(categoryModel.getCategory(name), before);
  });
});
