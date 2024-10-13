FROM node:hydrogen-bullseye-slim

WORKDIR /usr/src/app/server

COPY package*.json ./

RUN npm install

COPY . .

COPY .. /usr/src/app/

CMD ["npm", "run server"]