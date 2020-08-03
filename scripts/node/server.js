const Koa = require('koa');
const app = new Koa();
const path = require('path')
const views = require('koa-views');
var router = require('koa-router')();
const fs = require('fs');
// var echart = require('echarts');
app.use(views(path.join(__dirname, './views'), {
    extension: 'ejs'
}))
app.use(router.routes()).use(router.allowedMethods());
app.listen(3000);
var f = length => Array.from({
    length
}).map((v, k) => k * 1);

var series = [];
var xAxis = [];

router.get('/', async (ctx) => {
    // var xAxis = ["衬衫", "羊毛衫", "雪纺衫", "裤子", "高跟鞋", "袜子"];
    // var series = [5, 20, 200, 10, 10, 20];

    await ctx.render('index', {
        xAxis,
        series
    });

})

function ConvertToTable(data, callBack) {
    data = data.toString();
    var table = new Array();
    var rows = new Array();
    rows = data.split("\r\n");
    for (var i = 0; i < rows.length; i++) {
        table.push(rows[i].split(","));
    }
    callBack(table);
}

var refresh = function () {
    fs.readFile('D:\\work\\sim\\ACMSIMC_TUT\\build\\algorithm.dat', function (err, data) {
        if (err) {
            return console.error(err);
        }
        if (data.length < 2000) {
            return;
        }
        ConvertToTable(data, function (table) {
            // console.log(table);
            var length = table.length;
            var step = 4;
            series = [];
            xAxis = f(length / step);
            for (j = 0; j < table[1].length; j++) {
                var row = [];
                for (var i = 1; i < length / step; i += 1) {
                    row.push(table[i * step][j]);
                }
                series.push(row);
            }
            console.log("Visit http://localhost:3000")
            console.log("Refresh!");
        })
    });
}

var openexplore = function () {
    var cmd = 'start "%ProgramFiles%\Internet Explorer\iexplore.exe"'
    let child_process = require('child_process'),
        url = 'http://' + "localhost:3000";

    if (process.platform == 'win32') {
        cmd = 'start "%ProgramFiles%\Google\Chrome\Application\chrome.exe"';
    } else if (process.platform == 'linux') {
        cmd = 'xdg-open';
    } else if (process.platform == 'darwin') {
        cmd = 'open';
    }
    child_process.exec(`${cmd} "${url}"`);
    console.log("Opening" + cmd);
}
var timer = null;
fs.watch('D:\\work\\sim\\ACMSIMC_TUT\\build\\algorithm.dat', function (curr, prev) {
    if (Date.parse(prev.ctime) == 0) {
        console.log("文件被创建");
    } else if (Date.parse(curr.ctime) == 0) {
        console.log("文件被删除");
    } else if (Date.parse(curr.mtime) != Date.parse(prev.mtime)) {
        console.log("文件被修改");
        if (timer === null) {
            timer = setTimeout(function () {
                refresh();
                openexplore();
                timer = null;
            }, 5000);
        }
    }
});

if (xAxis.length == 0) {
    refresh();
    openexplore();
}