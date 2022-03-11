const Router = require('express');
const router = new Router();
const ratesController = require('../controllers/controller')


router.post('/savefile', ratesController.compile)

module.exports = router;