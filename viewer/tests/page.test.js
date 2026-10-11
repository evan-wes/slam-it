// The page's own files: the three.js it loads is the one its types describe.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

test("the CDN's three.js is the version of the types the 3D view is checked against", async () => {
  const page = await readFile(new URL("../index.html", import.meta.url), "utf8");
  const tools = JSON.parse(await readFile(new URL("../package.json", import.meta.url), "utf8"));
  const loaded = [...page.matchAll(/three@([\d.]+)\//g)].map((match) => match[1]);
  assert.ok(loaded.length > 0, "the import map names three.js's version");
  for (const version of loaded) {
    assert.equal(version, tools.devDependencies["@types/three"]);
  }
});
