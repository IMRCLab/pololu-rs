const express = require('express');
const path = require('path');
const fs = require('fs');
const yaml = require('js-yaml');

const app = express();
const port = 3000;

const configPath = path.join(__dirname, '..', 'config.yaml');
let config = {};

try {
  const fileContents = fs.readFileSync(configPath, 'utf8');
  config = yaml.load(fileContents);
  console.log("Loaded config:", config);
} catch (e) {
  console.error("Failed to load config.yaml:", e);
}

// Endpoint to provide config to frontend
app.get('/config', (req, res) => {
  res.json(config);
});

// Serve static files (your HTML)
app.use(express.static(path.join(__dirname, 'public')));

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});

