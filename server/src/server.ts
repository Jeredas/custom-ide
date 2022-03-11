require('dotenv').config()
const express = require('express');
const cors = require('cors');
const ratesRouter  = require('../src/router/router');
const bodyParser = require('body-parser');
const app = express()

const PORT = 6600;
async function start() {
    try {
        app.use(cors());
        app.use(bodyParser.json())
        app.use('', ratesRouter );
        app.listen(PORT, () => {
            console.log('Server is listening on PORT:' + PORT);
        });
    } catch (e) {
        console.log(e)
    }
}
start()