// https://medium.com/ms-club-of-sliit/jwt-bearer-token-authentication-for-express-js-5e95bf4dead0
import express from "express";
import jwt from 'jsonwebtoken'
const PORT = process.env.PORT ?? 1234

const app = express()
//app.use(bodyParser.json()); // support json encoded bodies
app.use(express.urlencoded({ extended: true })); // support encoded bodies
app.disable('x-powered-by')

app.use(express.json())


app.get('/test', (req, res) => {
  res.json({name:'ditto',id:'137'})
})

app.post('/token/provide/RESISTO-API', (req, res) => {
  // req.body deberíamos guardar en bbdd
  const { username, password } = req.body;
  console.log(username+'--'+password)
  //Send the response back
      //Mock user
      const user = {
        id:Date.now(),
        userEmail:username,
        password:password
    }

    //send abpve as payload
    jwt.sign({user},'secretkey',(err,token)=>{
        res.json({access_token:token,token_type:"bearer"
        })
    })
  //res.send(username + ' ' + password);
})

app.post('/resultado_mision',verifyToken, (req, res) => {
    // req.body deberíamos guardar en bbdd
    const {mission_id,resolution_code,resultados} = req.body
    console.log(mission_id+"--"+resolution_code)
    console.log(resultados)
    jwt.verify(req.token,'secretkey',(err,authData)=>{
        if(err)
            res.sendStatus(403);
        else{
            res.json({
                message:"Welcome to Profile",
                userData:authData
            })
           
        }
    })
  })
  
//Verify Token
function verifyToken(req,res,next){
    //Auth header value = > send token into header

    const bearerHeader = req.headers['authorization'];
    //check if bearer is undefined
    if(typeof bearerHeader !== 'undefined'){

        //split the space at the bearer
        const bearer = bearerHeader.split(' ');
        //Get token from string
        const bearerToken = bearer[1];

        //set the token
        req.token = bearerToken;

        //next middleweare
        next();

    }else{
        //Fobidden
        res.sendStatus(403);
    }

}


// la última a la que va a llegar
app.use((req, res) => {
  res.status(404).send('<h1>404</h1>')
})

app.listen(PORT, () => {
  console.log(`server listening on port http://localhost:${PORT}`)
})