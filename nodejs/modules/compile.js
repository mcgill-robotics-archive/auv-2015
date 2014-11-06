var jade = require('jade');
var stylus = require('stylus');
var fs = require('fs');
var path = require('path');
var chokidar = require('chokidar');
var helpers = require('./helpers');

function traverse(dir, options, callback) {
    callback.results = callback.results || [];
    if (options.exts) {
        if (helpers.isArray(options.exts)) {
            options.exts.forEach(function(ext) {
                callback.results[ext] = callback.results[ext] || [];
            });
        }
    }
    options.counter = (options.counter) || 1;
    options.dcounter = (options.dcounter) || 1;
    options.fcounter = (options.fcounter) || 0;
    options.ccounter = (options.ccounter) || 1;
    fs.readdir(dir, function(err, files) {
        if (err) {
            callback(err);
            return;
        }
        options.ccounter--;
        files.forEach(function(file) {
            var filepath = path.join(dir, file);
            options.counter++;
            fs.stat(filepath, function(err, stat) {
                if (err) {
                    callback(err);
                    return;
                }
                if (stat.isFile()) {
                    options.fcounter++;
                    if (helpers.filterExts(options.exts, filepath)) {
                        options.exts.forEach(function(ext) {
                            if (helpers.isExt(ext, filepath)) {
                                callback.results[ext].push(filepath);
                            }
                        });
                    }
                } else if (stat.isDirectory()) {
                    options.dcounter++;
                    options.ccounter++;
                    traverse(filepath, options, callback);
                }
                if (options.counter === options.dcounter + options.fcounter - options.ccounter) {
                    callback(null, callback.results);
                }
            });
        });
    });
}

function compileJade(file, options, callback) {
    var name = file.substr(0, file.indexOf('.'));
    name += '.html';
    var html = (jade.compileFile(file, options))({});
    fs.writeFile(name, html, function(err) {
        if (err) {
            callback(err);
            return;
        }
    });
}

function compileStyl(file, options, callback) {
    var readOptions = {
        encoding: 'utf8'
    };
    var name = file.substr(0, file.indexOf('.'));
    name += '.css';
    fs.readFile(file, readOptions, function(err, str) {
        stylus.render(str, function(err, css) {
            if (err) {
                callback(err);
                return;
            }
            fs.writeFile(name, css, function(err) {
                if (err) {
                    callback(err);
                    return;
                }
            });
        });
    });
}

function compileJades(files, options, callback) {
    files.forEach(function(file) {
        compileJade(file, options, callback);
    });
}

function compileStyls(files, options, callback) {
    files.forEach(function(file) {
        compileStyl(file, options, callback);
    });
}

function onChangeJade(file, options) {
    compileJade(file, options, function(err) {
        if (err) {
            console.error('There was a jade file saving error: ', err);
            return;
        }
    });
}

function onChangeStyl(file, options) {
    compileStyl(file, options, function(err) {
        if (err) {
            console.error('There was a stylus file saving error: ', err);
            return;
        }
    });
}

function watchFiles(files, options, callback) {
    var chokidarOpts = {
        persistent: true,
    };
    files.forEach(function(file) {
        chokidar.watch(file, chokidarOpts).on('all', function(evt, filename) {
            if (evt === 'change') {
                callback(filename, options);
            }
        });
    });
}

module.exports = function(dir, dev) {
    var jades = [],
        styls = [];
    var traverseOpts = {
        exts: ['jade', 'styl'],
    };
    var jadeOptions = {
        pretty: dev ? true : false,
    };
    var stylOptions = {};

    var dirName = __dirname.substr(0, __dirname.indexOf('/modules'));
    dirName = path.join(dirName, dir);

    traverse(dirName, traverseOpts, function(err, results) {
        if (err) {
            console.error('There was a traversal error:', err);
            return;
        }
        jades = results.jade;
        styls = results.styl;

        compileJades(jades, jadeOptions, function(err) {
            if (err) {
                console.error('There was a jade file saving error: ', err);
                return;
            }
        });

        compileStyls(styls, stylOptions, function(err) {
            if (err) {
                console.error('There was a stylus file saving error: ', err);
                return;
            }
        });

        if (dev) {
            watchFiles(jades, jadeOptions, onChangeJade);
            watchFiles(styls, stylOptions, onChangeStyl);
        }
    });
};
