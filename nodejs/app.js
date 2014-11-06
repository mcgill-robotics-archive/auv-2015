/*
 * Express and middleware
 */
var express = require('express');
var stylus = require('stylus');
var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');
var debug = require('debug')('ToolsFrontend');
var http = require('http');
var socketio = require('socket.io');

/*
 * Modules
 */
var compiler = require('./modules/compile');

/*
 * Get routes
 */
var routes = require('./routes/index');

/*
 * Server vars
 */
var usersConnected = 0;

/*
 * Server init
 */
var app = express();
var httpServer = http.Server(app);
var io = socketio(httpServer);

/*
 *Compile step
 */
compiler('public/components', app.get('env') === 'development');

/*
 * View Engine Setup
 */
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

/*
 * Style Engine Setup
 */
app.use(stylus.middleware(path.join(__dirname, 'public')));

/*
 * Favicon
 */
app.use(favicon(__dirname + '/public/img/favicon.gif'));

/*
 * Log requests
 */
app.use(logger('dev'));

/*
 * Parser middleware
 */
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({
    extended: false
}));
app.use(cookieParser());

/*
 * Append directories to base path
 */
app.use(express.static(path.join(__dirname, 'public')));
app.use(express.static(path.join(__dirname, 'bower_components')));

/*
 * Socket.io connection
 */
var userIO = function(socket) {
    console.log(++usersConnected + ' users connected');
    socket.on('disconnect', function() {
        console.log(--usersConnected + ' users connected');
    });
};
io.on('connection', userIO);

/*
 * Timeout
 */
app.use(function(req, res, next) {
    res.setTimeout(120000, function() {
        console.log('Request has timed out.');
        res.send(408);
    });
    next();
});

/*
 * Server set routes
 */
app.use('/', routes);

// catch 404 and forward to error handler
app.use(function(req, res, next) {
    var err = new Error('Not Found');
    err.status = 404;
    next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
    app.use(function(err, req, res, next) {
        res.status(err.status || 500);
        res.render('errorDebug', {
            message: err.message,
            error: err
        });
    });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
        title: 'Error'
    });
});

app.set('port', process.env.PORT || 8080);

var server = httpServer.listen(app.get('port'), function() {
    debug('Express server listening at address ' + server.address().address + ' on port ' + server.address().port);
});
