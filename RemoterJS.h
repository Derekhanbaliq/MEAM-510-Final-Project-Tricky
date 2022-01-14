/*
 * HTML and Javascript code
 */
const char body[] PROGMEM = R"===(

<!DOCTYPE html>
<html>

<meta charset="utf-8"/>

<style>
    body{
        background-color: #E1FFFF;
        font-family: Arial, Helvetica, sans-serif;
        touch-action: pan-x pan-y;
        -webkit-touch-callout: none;
  		-webkit-user-select: none;
   		-khtml-user-select: none;
      	-ms-user-select: none;
      	user-select: none;
    }
    h1{
        color: navy;
        font-size: 40px;
        text-align: center;
    }
    .button{
        background-color: #4CAF50;
        color: white;
        border: none;
        border-radius: 20px;
        text-align: center;
        font-size: 35px;
        width: 100px;
        height: 100px;
        font-weight: bold;
        position: absolute;
        align-items:center;
    }
    .button:hover{background-color: #4CAF50;}
    .button:active {background-color: #AA0000;}
    .b1pos{top: 125px; left: 250px;}
    .b2pos{top: 375px; left: 250px;}
    .b3pos{top: 250px; left: 125px;}
    .b4pos{top: 250px; left: 375px;}
    .b5pos{top: 175px; right: 150px;}
    .b6pos{top: 325px; right: 150px;}
    .b7pos{top: 125px; right: 350px;}
    .b8pos{top: 250px; right: 350px;}
    .b9pos{top: 375px; right: 350px;}
    .b10pos{top: 250px; left: 250px;}
    .dot{
    	position: absolute;
        align-items:center;
        top: 100px;
        left: 97.5px;
        height: 400px;
        width: 400px;
        background-color: azure;
        border-radius: 50%;
        border-style: solid;
  		border-color: gray;
	}
    .square1{
        position: absolute;
        align-items:center;
        top: 100px;
        right: 122.5px;
        height: 400px;
        width: 150px;
        background-color: azure;
        border-radius: 20px;
        border-style: solid;
  		border-color: gray;
	}
    .square2{
        position: absolute;
        align-items:center;
        top: 100px;
        right: 322.5px;
        height: 400px;
        width: 150px;
        background-color: azure;
        border-radius: 20px;
        border-style: solid;
  		border-color: gray;
	}
</style>

<body>
    <h1>Group 16 Remoter</h1>
    <p>
    <span class="dot"></span>
    <div class="square1"></div>
    <div class="square2"></div>
    <button class="button b1pos" id="b1" onpointerdown="press1()" onpointerup="release1()" onpointerleave="release1()"> F </button>
    <button class="button b3pos" id="b3" onpointerdown="press3()" onpointerup="release3()" onpointerleave="release3()"> L </button>
    <button class="button b4pos" id="b4" onpointerdown="press4()" onpointerup="release4()" onpointerleave="release4()"> R </button>
    <button class="button b2pos" id="b2" onpointerdown="press2()" onpointerup="release2()" onpointerleave="release2()"> B </button>
    <button class="button b5pos" id="b5" onpointerdown="click5()" onpointerup="release5()" onpointerleave="release5()"> 抓 </button>
    <button class="button b6pos" id="b6" onpointerdown="click6()" onpointerup="release6()" onpointerleave="release6()"> 放 </button>
    <button class="button b7pos" id="b7" onpointerdown="click7()" onpointerup="release7()" onpointerleave="release7()"> 墙 </button>
    <button class="button b8pos" id="b8" onpointerdown="click8()" onpointerup="release8()" onpointerleave="release8()"> 光 </button>
    <button class="button b9pos" id="b9" onpointerdown="click9()" onpointerup="release9()" onpointerleave="release9()"> 罐 </button>
    <button class="button b10pos" id="b10" onpointerdown="click10()" onpointerup="release10()" onpointerleave="release10()"> 控 </button>
    </p>
</body>

<script>
    document.addEventListener('keydown', (event) => 
    {
        var code=event.keyCode;
        if(code=='38'){press1();}
        else if(code=='40'){press2();}
        else if(code=='37'){press3();}
        else if(code=='39'){press4();}
        else if(code=='57'){click5();}
        else if(code=='48'){click6();}
        else if(code=='49'){click7();}
        else if(code=='50'){click8();}
        else if(code=='51'){click9();}
        else if(code=='27'){click10();}
	}, false);
	
    document.addEventListener('keyup', (event) =>
	{
        var code=event.keyCode;
        if(code=='38') {release1();}
        else if(code=='40'){release2();}
        else if(code=='37'){release3();}
        else if(code=='39'){release4();}
        else if(code=='57'){release5();}
        else if(code=='48'){release6();}
        else if(code=='49'){release7();}
        else if(code=='50'){release8();}
        else if(code=='51'){release9();}
        else if(code=='27'){release10();}
	}, false);

    function press1(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "press1", true);
        xhttp.send();
        document.getElementById('b1').style.backgroundColor = "#AA0000";
    }
    function press2(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "press2", true);
        xhttp.send();
        document.getElementById('b2').style.backgroundColor = "#AA0000";
    }
    function press3(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "press3", true);
        xhttp.send();
        document.getElementById('b3').style.backgroundColor = "#AA0000";
    }
    function press4(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "press4", true);
        xhttp.send();
        document.getElementById('b4').style.backgroundColor = "#AA0000";
    }
    function click5(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click5", true);
        xhttp.send();
        document.getElementById('b5').style.backgroundColor = "#AA0000";
    }
    function click6(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click6", true);
        xhttp.send();
        document.getElementById('b6').style.backgroundColor = "#AA0000";
    }
    function click7(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click7", true);
        xhttp.send();
        document.getElementById('b7').style.backgroundColor = "#AA0000";
    }
    function click8(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click8", true);
        xhttp.send();
        document.getElementById('b8').style.backgroundColor = "#AA0000";
    }
    function click9(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click9", true);
        xhttp.send();
        document.getElementById('b9').style.backgroundColor = "#AA0000";
    }
    function click10(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "click10", true);
        xhttp.send();
        document.getElementById('b10').style.backgroundColor = "#AA0000";
    }

    function release1(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b1').style.backgroundColor = "#4CAF50";
    }
    function release2(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b2').style.backgroundColor = "#4CAF50";
    }
    function release3(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b3').style.backgroundColor = "#4CAF50";
    }
    function release4(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b4').style.backgroundColor = "#4CAF50";
    }
    function release5(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b5').style.backgroundColor = "#4CAF50";
    }
    function release6(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b6').style.backgroundColor = "#4CAF50";
    }
    function release7(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b7').style.backgroundColor = "#4CAF50";
    }
    function release8(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b8').style.backgroundColor = "#4CAF50";
    }
    function release9(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b9').style.backgroundColor = "#4CAF50";
    }
    function release10(){
        var xhttp = new XMLHttpRequest();
        xhttp.open("GET", "release", true);
        xhttp.send();
        document.getElementById('b10').style.backgroundColor = "#4CAF50";
    }
</script>

</html>

)===";
