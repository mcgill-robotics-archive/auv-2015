var express = require('express');
var router = express.Router();

/*GET Arun's page*/
router.get('/', function(req, res){
    res.render('arun', { 
        title: 'Welcome'
    });
});

module.exports = router;
