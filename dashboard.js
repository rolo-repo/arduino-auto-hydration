/*
var jsdom = require("jsdom");
var JSDOM = jsdom.JSDOM;
global.document = new JSDOM("<body><table id='myTable'>\
								<tr>\
									<th>Date</th>\
									<th >Time</th>\
									<th>Temperature</th>\
								  <th>Action</th>\
								  <th></th>\
    </body>").window.document;
*/


//should not be

/*
should not appear
*/

/*
var connection = new WebSocket('ws://' + location.hostname + ':81/', ['arduino']);
connection.onopen = function () {
  connection.send('Connect ' + new Date());
};
connection.onerror = function (error) {
  console.log('WebSocket Error ', error);
};
connection.onmessage = function (e) {
  console.log('Server: ', e.data);
};
connection.onclose = function () {
  console.log('WebSocket connection closed');
};

function sendRGB () {
  var r = document.getElementById('r').value** 2 / 1023;
  var g = document.getElementById('g').value** 2 / 1023;
  var b = document.getElementById('b').value** 2 / 1023;

  var rgb = r << 20 | g << 10 | b;
  var rgbstr = '#' + rgb.toString(16);
  console.log('RGB: ' + rgbstr);
  connection.send(rgbstr);
}
*/

const temp_to_values = {
    0: "on",
    4: "off"
}

const values_to_temp = {
    "on" : 0,
    "off" : 1
}

function reportIR(){
    let statusBar = document.getElementById('statusBar'); 
    let el = document.createElement('STS');
    document.getElementById("temperature").style.backgroundColor
    el.innerHTML = '<span class=\'label label-default\'>OK</span>';
    el.style.backgroundColor = "blue";
    statusBar.appendChild(el);
}

function startTime(){
    let today = new Date();
    document.getElementById('datetime').innerHTML = today.toLocaleDateString() + " " + today.toLocaleTimeString();
    setTimeout(startTime, 900);
}

function updateSensorData(){
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            var regex = /Temp.*:(\d+.\d+).*Hum.*:(\d+.\d+)/;
            var val = regex.exec(this.responseText);
										
            console.log("T " + val[1]);
            console.log("H " + val[2]);
            //document.getElementById("temperature").innerHTML = this.responseText;}};
            document.getElementById("temperature").innerHTML = val[1] + ' C' + '&#176';
            document.getElementById("temperature").style.backgroundColor  = ( val[1] > 25 ) ? "red" : ( val[1] > 20 ) ? "green" : "orange";
            document.getElementById("humidity").innerHTML = val[2] +' &#37';
            document.getElementById("humidity").style.backgroundColor  = ( val[2] > 60 || val[2] < 15 ) ? "red" : ( val[2] > 30 ) ? "green" : "orange";
        }
    };
    xhttp.open("POST", "/", true);
    xhttp.send();
    setTimeout(updateSensorData, 20000);
}

function onActionButtonClick(url){
    let xhttp = new XMLHttpRequest();
    xmlhttp1.onreadystatechange = function() { 
        if (this.readyState == 4 && this.status == 200) {
            reportIR();
        }};
    xhttp.open('GET', url, true );
    xhttp.send();
}

var rowId  = 0;

function addRowArr(row_data) {
    var table = document.getElementById("myTable");
    /*
    var row = table.insertRow(-1);

   for (let i = 0 ; i < row_data.length ; i++)
    {
        let c = row.insertCell(i);
        c.innerHTML = row_data[i];
        c.setAttribute('name', rowId + '_' + i );
    }*/
  
    for (let i = 0 ; i < row_data.length ; i++) {
        var row = table.insertRow(-1);
        var cell1 = row.insertCell(0);
        var cell2 = row.insertCell(1);
       // var cell3 = row.insertCell(2);
        var cell4 = row.insertCell(2);
       
        var cell1Val = document.createElement("INPUT");
        var cell2Val = document.createElement("INPUT");
       // var cell3Val = document.createElement("INPUT");
        var cell4Val = document.createElement("INPUT");
       
       

        row.setAttribute('name', table.rows.length);
        row.classList.add("tstyle");

        cell1Val.readOnly = true;
        cell1Val.value = row_data[i].date;
        if (row_data[i].recuring == 'false') {
            cell1Val.setAttribute('name', 'd_' + table.rows.length);
        }

        cell2Val.readOnly = true;
        cell2Val.value = row_data[i].time; 
        cell2Val.setAttribute('name', 't_' + table.rows.length);

       // cell3Val.readOnly = true;
        //cell3Val.value = temp_to_values[row_data[i].code];
        //cell3Val.setAttribute('name', 'temp_' + table.rows.length);

        cell4Val.readOnly = true;
        cell4Val.value = ((row_data[i].code == 0 ) ? "on" : "off");
        cell4Val.setAttribute('name', 'a_' + table.rows.length);

        cell1.appendChild(cell1Val);
        cell2.appendChild(cell2Val);
      //  cell3.appendChild(cell3Val);
        cell4.appendChild(cell4Val);

        console.log(row.innerHTML);
        console.log(row.outerHTML);
    }
 
    updateTable(table);
}

function updateSchedule() {
    var xmlhttp1 = new XMLHttpRequest();

    xmlhttp1.onreadystatechange = function () {
        if (this.readyState == 4 && this.status == 200) {
            var myArr = JSON.parse(this.responseText);
            addRowArr(myArr);
        }
    };
    xmlhttp1.open("GET", "/schedule", true);
    xmlhttp1.send();
}
/*
addRowArr(['2009-12-12', '10:00', 1]);
addRowArr(['2009-12-12', '12:00', 0]);
addRowArr(['2009-12-12', '11:00', 0]);

addRowArr(['2009-12-12', '13:00', 1]);
addRowArr(['2009-12-12', '14:00', 0]);

console.log(document.getElementById("myTable").innerHTML);*/

function addRow() {
    var table = document.getElementById("myTable");
  /*  var lastRow = table.rows[table.rows.length - 1];
    if ( table.rows.length >= 2 )
        lastRow.cells[4].innerHTML='<button type="button" class="btn btn-info btn-md" onclick="deleteRow(this)">-</button>';
    else
        table.rows[0].cells[4].innerHTML='';
        
    console.log(lastRow.cells[0].firstChild);
    */
    var row = table.insertRow();
    var cell1 = row.insertCell(0);
    var cell2 = row.insertCell(1);
   // var cell3 = row.insertCell(2);
    var cell4 = row.insertCell(2);
    row.classList.add("tstyle");


     rowId = table.rows.length;

     let n = new Date();
     let y = n.getFullYear();
     let m = n.getMonth() + 1;
     let d = n.getDate();
     var date = '' + y + '-' + ((m < 10) ? '0' + m : m) + '-' + ((d < 10) ? '0' + d : d);

     cell1.innerHTML = ''.concat('<div class="checkbox"><label><input type="checkbox" onclick="getElementsByName(\'d_', rowId, '\')[0].disabled=!getElementsByName(\'d_', rowId, '\')[0].disabled;"><input type="date" value="', date, '" name="d_', rowId, '" min="00:00" max="24:00" required></label><div>');
     cell2.innerHTML = ''.concat('<input type="time" value="12:00" name="t_', rowId, '" min="00:00" max="24:00" required>');
    // cell3.innerHTML = ''.concat('<select name="temp_', rowId, '"><option class="cold" value="0">c_low</option><option class="cold" value="1">c_high</option><option class="hot" value="2">h_low</option><option class="hot" value="3">h_high</option></select>');
     cell4.innerHTML = ''.concat('<label class="radio-inline"><input type="radio" name="a_', rowId, '" value="on">On</label><label class="radio-inline"><input type="radio" name="a_', rowId, '" value="off">Off</label>');
     //cell5.innerHTML = '<button type="button" class="btn btn-info btn-md" onclick="addRow()">+</button>';

     updateTable(table);
     return row;
}

function deleteRow(btn) {
    var row = btn.parentNode.parentNode;
    if ( row.parentNode.childNodes.length > 1 ) {
        row.parentNode.removeChild(row);}				
    updateTable();
}

function doSend(){
    var table = document.getElementById("myTable");
    for ( let row of table.rows) {
        row.cells[0].firstChild.disabled = false

        if (isNaN(row.cells[2].firstChild.value)) {
            row.cells[2].firstChild.value = values_to_temp[ row.cells[2].firstChild.value ];
        }
    }



    console.log(document.getElementById("form_id").innerHTML);
    document.getElementById("form_id").submit();
}

function updateTable(table) {
    table = ( null == table ) ?  document.getElementById("myTable"): table;

    var c_colls = table.rows[0].cells.length;

    for  ( let row of table.rows ) {
        if ( c_colls > row.cells.length) {
            let c = row.insertCell(-1);
            c.innerHTML = '<button type="button" class="btn btn-info btn-md" onclick="deleteRow(this)">-</button>';
        }
    };

    console.log(c_colls);
}

google.charts.load('current', {packages: ['table']});     
google.charts.load('current', {packages: ['corechart','line']});  
google.charts.setOnLoadCallback(updateGraph);

function updateGraph()
{
    var xmlhttp1 = new XMLHttpRequest();
				
    xmlhttp1.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {   	
            var myArr = JSON.parse(this.responseText);
            drawChart(myArr);
        }
    };
    xmlhttp1.open("GET", "/bme", true);
    xmlhttp1.send();
					
    setTimeout(updateGraph, 240000);
}
					
// Draw the chart and set the chart values
function drawChart(arr) {
    var data = new google.visualization.DataTable();
    data.addColumn('string', 'metric');
    data.addColumn('number', 'C');
    data.addColumn('number', '%');
    //data.addColumn('number', '%');
    for ( let i =0 ; i < arr.length ; i++ )
    {
        data.addRows([[ 't_' + i, arr[i].t, arr[i].h] ]);
    }
						
    // Optional; add a title and set the width and height of the chart
    var options = {'title':'Average environment measurment'};
    //{left:20,top:0,width:'50%',height:'75%'}
    // Display the chart inside the <div> element with id="piechart"
    var chart = new google.visualization.LineChart(document.getElementById('chart'));
    chart.draw(data, options);
}			
