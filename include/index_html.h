#pragma once
const char indexHTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <head>
    <title>PMX STATUS</title>
    <meta http-equiv="content-type" content="text/html; charset=UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=yes">
    <script type="text/javascript">
      var Url = "/";     
      window.onload = start;
      
      function start (){
        myAjax = new XMLHttpRequest();
        myAjax.onreadystatechange=LesenAjax;
        setInterval(aktualisieren,2000);
        aktualisieren ();
      }
      
      function aktualisieren (){
        myAjax.open("GET",Url+"?status",true);
        myAjax.send();
      }
      
      function LesenAjax(){
        if (myAjax.readyState==4 && myAjax.status==200){
          var str=myAjax.responseText;
          if(str!='OK'){
            document.getElementById('stage').innerHTML=str;
          }
        }
      }
      
      
      function httpGet(sendthis){
        myAjax.open("GET",Url+"?"+sendthis,true);
        myAjax.send();
      }
    </script>
    
    <style type="text/css">
      html,body {
        width:100%;
        height:100%;
        overflow:hidden;
      }
      
      body {
        font-family:calibri,arial,helvetica,sans-serif;
        font-size:14px;
        color:#222;
        background-color:#ddd;
        padding:0;
        margin:0;
        display: flex;
        justify-content: center;
        align-items: center;
      }
      
      #deadcenter{
        white-space:nowrap;
        background:#f6f6f6;
        padding:0px 20px;
        border-radius:8px;
        box-shadow: 0px 0px 34px 1px rgba(0,0,0,0.31);
        text-align:center;
      }
      
      h1{
        text-transform:uppercase;
      }
      
      .bigInfoThingy{
        color: white;
        padding: 10px 20px;
        margin: 10px 0;
        border-radius: 12px;
        background-color:#ccc;
      }
      
      .bigInfoThingy p{
        padding: 0;
        margin: 0;
      }
      
      .bigWords{
        font-weight:bold; 
        font-size:300%;
      }
      
      #countdown{
        color:#6b6b6b;
      }
      
      #butgrid{
        margin:2em 0;
        display:flex;
        justify-content:space-between;
      }
      
      button{
        padding: 20px 35px;
        font-size:180%;
        border:none;
        border-radius: 0.3em;
        color:white;
        font-weight: bold;
        cursor:pointer;
      }
      
      #stage{
        color:#535353;
      }
      
      #copyRight{
        position:absolute;
        bottom:10px;
        right:10px;
        z-index:-99;
      }
      
      #copyRight a{
        font-size:95%;
        color:#666;
        text-decoration:none;
        border:none;
      }
      
      #copyRight a:hover{
        text-decoration:none;
        border:none;
        color:black;
      }
      
      #settingsBut{
        padding:0;
        margin:0;
        color:black;
        background-color:none;
        position: relative;
        right: -5px;
        top: 5px;
        font-size: 200%;
        display: block;
        float: right;
        text-decoration:none;
        cursor:pointer;
      }
      
      #settingsBut:hover{
        color:red;
      }
      
    </style>
  </head>
  <body>
    <div id="deadcenter">
      <h1>HSD UMT<br>PMX Live Data</h1>
      
      <div id="stage">
        <div class="bigInfoThingy bigWords">Loading...</div>
      </div>
    </div>
    <div id="copyRight">
      <a href="https://mv.hs-duesseldorf.de/studium/Lehrgebiete/physik_und_umwelttechnik" target="_blank">HSD UMT 2019-21</a>
    </div>
  </body>
</html>
)=====";
