var spawn = require('child_process').spawn,
    ls    = spawn('python',['python_serial.py']);

ls.stdout.on('data', function (data) {
    console.log('stdout: ' + data);
});

ls.stderr.on('data', function (data) {
    console.log('stderr: ' + data);
});
