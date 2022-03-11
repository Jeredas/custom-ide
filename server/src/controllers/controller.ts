const ratesService = require('../services/apiService');

class RatesController {
    public async compile(req,res) {
        try { const converted = await ratesService.compile(req.body.code)
            res.json(converted);
        } catch(e) {
            console.log(e);
        }
    }
}
 

module.exports = new RatesController()