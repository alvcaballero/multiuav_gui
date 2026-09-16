// https://medium.com/ms-club-of-sliit/jwt-bearer-token-authentication-for-express-js-5e95bf4dead0
import express from 'express';
import jwt from 'jsonwebtoken';

const PORT = process.env.PORT ?? 1234;

const app = express();
app.use(express.urlencoded({ extended: true })); // support encoded bodies
app.disable('x-powered-by');
app.use(express.json());

app.post('/token', (req, res) => {
  // req.body must  save a DB
  const { username, password } = req.body;
  console.log(username + '--' + password);
  //Send the response back
  //Mock user
  const user = {
    id: Date.now(),
    userEmail: username,
    password: password,
  };
  //send abpve as payload
  jwt.sign({ user }, 'secretkey', (err, token) => {
    res.json({ access_token: token, token_type: 'bearer' });
  });
  //res.send(username + ' ' + password);
});

app.post('/drones/mission/start', verifyToken, (req, res) => {
  // req.body deberíamos guardar en bbdd
  console.log('\x1b[43m%s\x1b[0m', 'mission Start');

  console.log(req.body);
  if (req.body.hasOwnProperty('routes')) {
    console.log(req.body.routes);
    req.body.routes.map((route) => {
      console.log(route);
    });
  }
  jwt.verify(req.token, 'secretkey', (err, authData) => {
    if (err) res.sendStatus(403);
    else {
      console.log('login');
      res.json({
        message: 'Welcome to Profile',
        userData: authData,
      });
    }
  });
});

app.post('/drones/mission/result', verifyToken, (req, res) => {
  // req.body deberíamos guardar en bbdd
  console.log('\x1b[43m%s\x1b[0m', 'mission result');
  const { mission_id, resolution_code } = req.body;
  console.log(`mission_id ${mission_id} resolution code ${resolution_code}`);
  jwt.verify(req.token, 'secretkey', (err, authData) => {
    if (err) res.sendStatus(403);
    else {
      console.log('login');
      res.json({
        message: 'Welcome to Profile',
        userData: authData,
      });
    }
  });
});

app.post('/drones/mission/media', verifyToken, (req, res) => {
  console.log('\x1b[43m%s\x1b[0m', 'mission Media');
  const { mission_id, result, files } = req.body;
  console.log(`mission_id ${mission_id} result ${result} `);
  console.log('files');
  console.log(files);
  console.log('resultados');
  console.log(result);
  jwt.verify(req.token, 'secretkey', (err, authData) => {
    if (err) res.sendStatus(403);
    else {
      console.log('login');
      res.json({
        message: 'Welcome to Profile',
        userData: authData,
      });
    }
  });
});

//Verify Token
function verifyToken(req, res, next) {
  //Auth header value = > send token into header
  const bearerHeader = req.headers['authorization'];
  //check if bearer is undefined
  if (typeof bearerHeader !== 'undefined') {
    //split the space at the bearer
    const bearer = bearerHeader.split(' ');
    //Get token from string
    const bearerToken = bearer[1];
    //set the token
    req.token = bearerToken;
    //next middleweare
    next();
  } else {
    //Fobidden
    res.sendStatus(403);
  }
}

// la última a la que va a llegar
app.use((req, res) => {
  res.status(404).send('<h1>404</h1>');
});

app.listen(PORT, () => {
  console.log(`server listening on port http://localhost:${PORT}`);
});
