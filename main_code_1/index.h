/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-controls-car-via-web
 */

const char *HTML_CONTENT = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Document</title>
  <style>
    body{
      font-size: 24px;
      text-align: center;
    }
    h2{margin: 0%;
    margin-bottom: 10px;}
    .container{
      width: 85%;
      margin: auto;
      height:100vh ;
    }
    .main-section{
      display: flex;
      justify-content: space-around;
      align-items: center;
      height: 75vh;
    }
    .sec2-container{
      display: flex;
      justify-content: center;
      align-items: center;
    }
    .left-right{
      display: flex;
      justify-content: center;
      align-items: center;
    }
    .sec1{
      width:30%;
      height: 100%;
    }
    .directions{
      position: relative;
      height: 100%;
    }

    
    /* Buttons - start */
    .button_up, .button_down,.button_left, .button_right,.servo_up ,.servo_down,.servo_left,.servo_right{ width:150px; height:150px; cursor: pointer;}
    .button_up {
  background: url("https://i.ibb.co/8mC0mhv/upward-button.png") center center/cover no-repeat;
  position: absolute;
  left: 30%;
  top: 0%;

}
.button_up:active{
  background: url("https://i.ibb.co/FXYyqcn/upward-button-active-Custom.png") center center/cover no-repeat;
}

.button_down {
  
  background: url("https://i.ibb.co/mGGbYYr/downward-button.png") center center/cover no-repeat;
  position: absolute;
  left: 30%;
  top: 40%;
  
}
.button_down:active{
  background: url("https://i.ibb.co/9nGn4pN/downward-button-active.png") center center/cover no-repeat;
}
.button_right {
  
  background: url("https://i.ibb.co/s9XNLRF/right-button.png") center center/cover no-repeat;   
  position: absolute;
  left: 60%;
  top: 20%;
}

.button_right:active{
  background: url("https://i.ibb.co/Kxd2jyP/right-button-active-Custom.png") center center/cover no-repeat;
}

.button_left {
  
  background: url("https://i.ibb.co/hBL5wSr/left-button.png") center center/cover no-repeat;
  position: absolute;
  left: 0%;
  top: 20%;
}
.button_left:active{
  background: url("https://i.ibb.co/HY4J3Tv/left-button-active-Custom.png") center center/cover no-repeat;
}

/* Buttons - End */

/* Servo Start */
.servo_up{
  
  background: url("https://i.ibb.co/8mC0mhv/upward-button.png") center center/cover no-repeat;
   margin: 10px;
}
.servo_up:active{
  background: url("https://i.ibb.co/FXYyqcn/upward-button-active-Custom.png") center center/cover no-repeat;
}

.servo_down{
  
  background: url("https://i.ibb.co/mGGbYYr/downward-button.png") center center/cover no-repeat;
   margin: 10px;
}
.servo_down:active{
  background: url("https://i.ibb.co/9nGn4pN/downward-button-active.png") center center/cover no-repeat;
}
.servo_left{
  
  background: url("https://i.ibb.co/hBL5wSr/left-button.png") center center/cover no-repeat;
   margin: 10px;
}
.servo_left:active{
  background: url("https://i.ibb.co/HY4J3Tv/left-button-active-Custom.png") center center/cover no-repeat;
}
.servo_right{
  background: url("https://i.ibb.co/s9XNLRF/right-button.png") center center/cover no-repeat;
   margin: 10px;
}
.servo_right:active{
  background: url("https://i.ibb.co/Kxd2jyP/right-button-active-Custom.png") center center/cover no-repeat;
}


/* Servo End */
  </style>
</head>
<body>
  <div class="container">
    <header><h1>Control Car via Web</h1></header>
    <main>
      <div class="main-section">
        <div class="sec1">
          <h2>Directions</h2>
          <div class="directions">
            <div id="2" class="button_left"></div>
            <div id="1" class="button_up""></div>
            <div id="3" class="button_down"></div>
            <div id="4" class="button_right"></div>
          </div><!--directions-->
        </div><!--sec1-->
        <div class="sec2">
          <h2>Servo</h2>
          <div class="sec2-container">
            <div class="up-down">
              <div id="5" class="servo_up""></div>
              <div id="6" class="servo_down"></div>
            </div>
            <div class="left-right">
              <div id="7" class="servo_left""></div>
              <div id="8" class="servo_right"></div>
            </div><!--left-right-->
            
          </div><!--sec2-container-->
        </div><!--sec2-->
      </div><!--main-section-->
    
    <hr>
    </main>
      <h3 id="10" class="toggle active">Automatic</h3>
    
    <button class="text" onclick="etoggle()">Click me to change to manual</button>

<p>
WebSocket : <span id="ws_state" style="color:blue">closed</span><br>
</p>
<button id="wc_conn" type="button" onclick="wc_onclick();">Connect</button>
<br>
<br>

  </div>

<script>
var ws = null;
let toggle=document.querySelector(".toggle")
  let text=document.querySelector(".text")

function init() 
{
  
  var container = document.querySelector("main");
    container.addEventListener("touchstart", mouse_down);
    container.addEventListener("touchend", mouse_up);
    container.addEventListener("touchcancel", mouse_up);
    container.addEventListener("mousedown", mouse_down);
    container.addEventListener("mouseup", mouse_up);
    container.addEventListener("mouseout", mouse_up);    
}
function init2() 
{
  
  var container = document.querySelector("#container2");
    container.addEventListener("touchstart", mouse_down);
    container.addEventListener("touchend", mouse_up);
    container.addEventListener("touchcancel", mouse_up);
    container.addEventListener("mousedown", mouse_down);
    container.addEventListener("mouseup", mouse_up);
    container.addEventListener("mouseout", mouse_up);    
}
function ws_onmessage(e_msg)
{
    e_msg = e_msg || window.event; // MessageEvent
 
    //alert("msg : " + e_msg.data);
}
function ws_onopen()
{
  document.getElementById("ws_state").innerHTML = "OPEN";
  document.getElementById("wc_conn").innerHTML = "Disconnect";
}
function ws_onclose()
{
  document.getElementById("ws_state").innerHTML = "CLOSED";
  document.getElementById("wc_conn").innerHTML = "Connect";
  console.log("socket was closed");
  ws.onopen = null;
  ws.onclose = null;
  ws.onmessage = null;
  ws = null;
}
function wc_onclick()
{
  if(ws == null)
  {
    ws = new WebSocket("ws://" + window.location.host + ":81");
    document.getElementById("ws_state").innerHTML = "CONNECTING";
    
    ws.onopen = ws_onopen;
    ws.onclose = ws_onclose;
    ws.onmessage = ws_onmessage; 
  }
  else
    ws.close();
}
function mouse_down(event) 
{
  if (event.target !== event.currentTarget) 
  {
    var id = event.target.id;
    if(toggle.innerHTML==='Automatic'){
      send_command(id+'1');
    console.log(id+'1')
    }else{
      send_command(id+'0');
    console.log(id+'0')
    }
    
   
    // event.target.style.backgroundImage = "url('https://esp32io.com/images/tutorial/" + img_name_lookup[id] + "_active.png')";
   
    }
    event.stopPropagation();    
    event.preventDefault();    
}

function mouse_up(event) 
{
  if (event.target !== event.currentTarget) 
  {
    var id = event.target.id;
    send_command(0);
    // event.target.style.backgroundImage = "url('https://esp32io.com/images/tutorial/" + img_name_lookup[id] + "_inactive.png')";
    }
    event.stopPropagation();   
    event.preventDefault();    
}
function send_command(cmd) 
{   
  if(ws != null)
    if(ws.readyState == 1)
      ws.send(cmd + "\r\n");   
}


  
  function etoggle(){
    toggle.classList.toggle("active")

    if(toggle.classList.contains("active")){
      toggle.innerHTML="Automatic"
      text.innerHTML="Click me to change to manual"
      toggle.removeAttribute('id')
      toggle.setAttribute('id',9)
    // console.log(toggle.id)
    //   send_command(toggle.id)
    }
    else{
      toggle.innerHTML="Manual"
      text.innerHTML="Click me to change to Automatic"
      toggle.removeAttribute('id')
      toggle.setAttribute('id',10)
      // console.log(toggle.id)
      // send_command(toggle.id)
    }
  }



window.onload = function(){
  init()

  console.log('webpage loaded successfully')
  send_command(toggle.innerHTML)
  console.log(toggle.innerHTML)
}
</script>
</body>
</html>
)=====";