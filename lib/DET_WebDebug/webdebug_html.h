const char webDebugHTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <head>
    <title>Web Debug</title>
    <meta http-equiv="content-type" content="text/html; charset=UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=0.7, user-scalable=yes">
    <style type="text/css">
      html,body{
        width:100%;
        height:100%;
        overflow:hidden;
      }
      
      body{
        font-family:calibri,arial,helvetica,sans-serif;
        font-size:14px;
        color:#222;
        background-color:#cdcbcb;
        padding:0;
        margin:0;
      }

			h1{
				background-color:blue;
				color:white;
				margin:0;
				padding:5px;
				text-transform:uppercase;
			}

			#debug{
				width:calc(100% - 10px);
				height:calc(100% - 88px);
				overflow: auto;
				resize:none;
				margin:0;
				padding:5px;
        font-family:consolas,calibri,helvetica,sans-serif;
			}
			
			#debug::placeholder{
				color:#666;
				font-style:italic;
			}

			#footer{
				display:flex;
				justify-content:space-between;
			}

			a{
        font-size:95%;
        color:#666;
        text-decoration:none;
        border:none;
      }
      
      a:hover{
        text-decoration:none;
        border:none;
        color:black;
      }
			
			button{
				border:none;
				padding:8px 20px;
			}
			
			#pause{
				color:white;
				background-color:green;
			}
	</style>
    
  </head>
  <body>
		<h1>Web Debug</h1>
		
		<textarea id="debug" autofocus readonly placeholder="nothing here yet - connecting"></textarea>
		
		<div id="footer">
			<a href="https://detlefamend.de" target="_blank">Detlef Amend 2020</a>
			<div>
				<button onclick="recon()">Reconnect</button>
        <button onclick="clearDebug()">Clear</button>
				<button onclick="pauseDebug()" id="pause">Pause</button>
			</div>
		</div>

    <script>
			var websocket=new WebSocket('ws://' + location.hostname + ':1968/');
			var pauseScroll=false;
			
      function recon(){
        location.reload();
      }

      function clearDebug(){
        document.getElementById('debug').value='';
      }
			
			function pauseDebug(){
				pauseScroll=!pauseScroll;
				if(pauseScroll){
					document.getElementById('pause').innerHTML='Paused';
					document.getElementById('pause').style.backgroundColor='red';
				}else{
					document.getElementById('pause').innerHTML='Pause';
					document.getElementById('pause').style.backgroundColor='green';
				}
			}
      
      websocket.onmessage=function(e){
				if(pauseScroll){
					return;
				}
				document.getElementById('debug').value+=e.data;
				document.getElementById('debug').scrollTop=document.getElementById('debug').scrollHeight;
      };
    </script>
  </body>
</html>
)=====";
