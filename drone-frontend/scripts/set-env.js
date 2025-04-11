const fs = require('fs');
const dotenv = require('dotenv');
dotenv.config();

const envFilePath = './src/environments/environment.ts';
const fileContent = `
export const environment = {
  production: false,
  googleMapsApiKey: '${process.env.GOOGLE_MAPS_API_KEY}'
};
`;

fs.writeFileSync(envFilePath, fileContent);

