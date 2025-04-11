const fs = require('fs');
const dotenv = require('dotenv');
dotenv.config();

const key = process.env.GOOGLE_MAPS_API_KEY;

if (!key) {
  console.error("GOOGLE_MAPS_API_KEY not set in .env");
  process.exit(1);
}

const targetPath = './src/environments/environment.ts';

const fileContent = `
export const environment = {
  production: false,
  googleMapsApiKey: '${key}'
};
`;

fs.writeFileSync(targetPath, fileContent);
console.log(`✅ Wrote Google Maps API key to ${targetPath}`);
