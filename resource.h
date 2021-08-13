#ifndef resource_h__
#define resource_h__

// HTML web page to handle 3 input fields (input1, input2, input3)

const char response_html[] PROGMEM = R"rawliteral(
<br>
  <h1 style="text-align:center;">${data}</h1>
  <hr>
<br><a href="/dash">Return to Home Page</a>
)rawliteral";

const char success_html[] PROGMEM = R"rawliteral(
<br>
  <h1 style="text-align:center;">Thank You!</h1>
  <hr>
<br><a href="/">Return to Home Page</a>
)rawliteral";

const char landing_html[] PROGMEM = R"rawliteral(
<br>
  <h1 style="text-align:center;">Hello!</h1>
  <hr>
<br><a href="/">Return to Home Page</a>
)rawliteral";

const char restart_in_process_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
	<head>
		 <script  type="text/javascript">
		function countDownSec(value) {
         document.getElementById('countDownValue').innerHTML = value;
         setTimeout(countDownSec, 1000, (value > 0 ) ? --value : '<a href="/">Back</a>' );
		}</script>    
	</head>
	<body onload="countDownSec(${delay})">
		<div style="text-align: center"> 
			<h1>Going to restart in <span id="countDownValue"></span> sec</h1>
	     </div>
	</body>
	</html>
)rawliteral";

const char configure_ir_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<style>
table{
  font - family: arial, sans - serif;
  border - collapse: collapse;
}
td,th{
  border: 1px solid #dddddd;
  text - align: left;
  padding: 8px;
}
</style>
</head>
<body>
<h2>${action}</h2>
<table>
<form action="${action}">
<tbody>
  <tr style="background-color:blue">
  <td><input ${cold_high_disabled} style="background-color:${cold_high_color}" type="submit" id ="1" name="cold_high" value="Cold High"></td>
  <td><input ${cold_low_disabled} style="background-color:${cold_low_color}" type="submit" id ="2" name="cold_low" value="Cold Low"></td>
  </tr>
  <tr style = "background-color:red">
  <td><input ${hot_high_disabled} style="background-color:${hot_high_color}" type="submit" id="3" name="hot_high" value="Hot High"></td>
  <td><input ${hot_low_disabled} style="background-color:${hot_low_color}" type="submit" id ="4" name ="hot_low" value = "Hot Low"></td>
  </tr>
   </tbody>
   <tfoot>
  	<tr>
      	<td colspan="2"><input ${off_disabled} style="margin-left:32%" type="submit" id ="5" name ="off" value = "Turn Off"></td>
     </tr>
   </tfoot>
</form>
<table>
</form>
</body>
</html>
)rawliteral";


const char dashboard_html[] PROGMEM=R"rawliteral(
<!DOCTYPE html>
<html lang="en">
	<head>
		<title>Bootstrap Example</title>
		<meta charset="utf-8">
		<meta name="viewport" content="width=device-width, initial-scale=1">
		<link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.4.1/css/bootstrap.min.css">
		<script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script>
		<script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.4.1/js/bootstrap.min.js"></script>
		<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
        <script  type="text/javascript">${script_dashboard}</script>    
		<style>
			table, td ,th {
				border: 2px solid black;
				border-spacing: 5px;
			}
			th, td {
				padding: 5px;
			}
			.cold {
				background-color:#80FFFF
			}
			.hot {
				background-color:#FF7171
			}
			.tstyle {
				text-align: center;
			}
		</style>
	</head>
	<body onload="startTime();updateSchedule();updateSensorData();updateGraph();">
		<div class="jumbotron">
			<div style="text-align: center"><h2>HYDRATION</h2></div>
			<div class ="container-fluid">
				<div class="col-lg-6 col-md-6 col-sm-6 col-xs-6">	  
					<h4><label for="datetime">Date/Time:</label> <span id="datetime"></span></h4> 
				</div>
				<div  class="col-lg-6 col-md-6 col-sm-6 col-xs-6" style="text-align: right">build:${version}</div>
				<div  class="col-lg-12 col-md-12 col-sm-12 col-xs-12" id="statusBar">
						<span id="temperature" class="label label-default">Default Label</span>
						<span id="humidity" class="label label-default">Default Label</span>
				</div>
			 </div>
		</div>
			<div class ="container-fluid">
				<div class="col-lg-2 col-md-2 col-sm-6 col-xs-6">
					<div class="btn-block">
						<button type="button" class="btn btn-success btn-lg btn-block" onclick="let xhttp = new XMLHttpRequest();xhttp.onreadystatechange = function() { if (this.readyState == 4 && this.status == 200) {reportIR();}};xhttp.open('GET', '/run?on', true );xhttp.send();">ON</button>
						<button type="button" class="btn btn-success btn-lg btn-block" onclick="let xhttp = new XMLHttpRequest();xhttp.onreadystatechange = function() { if (this.readyState == 4 && this.status == 200) {reportIR();}};xhttp.open('GET', '/run?off', true );xhttp.send();">OFF</button>
					</div>
				</div>
				<div class="col-lg-4 col-md-4 col-sm-12 col-xs-12">
					<div style="width: 100%; height: 100%" id="chart"></div>
				</div>
			</div>
		<br>
		<div class="jumbotron">
			<div class ="container">
				<div  class="col-lg-6 col-md-6 col-sm-12 col-xs-12">
					<form action="fput" id="form_id">
						<table id="myTable" style="text-align: center">
<colgroup><col span="1" style="width:40%;"><col span="1" style="width:10%;"> <col span="1" style="width:20%;"><col span="1" style="width:30%;"></colgroup>
							<thead>
								<tr>
									<th class="tstyle">Date</th>
									<th class="tstyle">Time</th>
								    <th class="tstyle">Action</th>
								    <th class="tstyle"><button type="button" class="btn btn-info btn-md" onclick="addRow()">+</button></th>
								</tr>
							</thead>
							<tbody>${t}</tbody>
						</table>
					</form>
					<br>
					<div style="text-align: center">					
						<input  type="button" class="btn btn-info btn-md" value="Submit" onclick="doSend()"/>
					</div>
				</div>
			</div>
		</div>
</body>)rawliteral";
#endif // resource_h__
