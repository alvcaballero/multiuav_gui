import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { categoryModel, CategoryError } from '../models/category.js';

// The writes rewrite the git-tracked files of config/devices/interfaces/. These tests only exercise
// the paths that must NOT write (refused before the file is touched), so they never touch them.

const rejects = (promise, status) =>
  assert.rejects(promise, (err) => err instanceof CategoryError && err.status === status);

describe('categoryModel writes', () => {
  test('refuses to overwrite an existing category and leaves it untouched', async () => {
    const name = categoryModel.getAll()[0];
    const before = structuredClone(categoryModel.getCategory(name));

    await rejects(categoryModel.createCategory(name, { subscribers: {}, services: {} }), 409);

    assert.deepEqual(categoryModel.getCategory(name), before);
  });

  test('refuses to update or delete a category that does not exist', async () => {
    await rejects(categoryModel.updateCategory('no_such_category', {}), 404);
    await rejects(categoryModel.deleteCategory('no_such_category'), 404);
  });

  test('refuses a name that is not a key, so it can never become a path outside the folder', async () => {
    for (const name of ['../escape', 'a/b', 'with space', '']) {
      await rejects(categoryModel.createCategory(name, {}), 400);
    }
  });

  test('refuses an invalid body before writing, with the path of every problem', async () => {
    const name = categoryModel.getAll()[0];
    const before = structuredClone(categoryModel.getCategory(name));
    const body = { subscribers: { position: { nmae: '/gps', messageType: 'sensor_msgs/NavSatFix' } } };

    await assert.rejects(categoryModel.updateCategory(name, body), (err) => {
      assert.equal(err.status, 400);
      assert.deepEqual(
        err.errors.map((e) => e.path),
        [[name, 'subscribers', 'position']]
      );
      return true;
    });
    assert.deepEqual(categoryModel.getCategory(name), before);
  });
});
