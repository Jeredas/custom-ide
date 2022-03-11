import React, { useState } from "react";
import logo from "./logo.svg";
import "./App.css";
import AceEditor from "react-ace";
import "ace-builds/webpack-resolver";
import "ace-builds/src-noconflict/mode-c_cpp";
import "ace-builds/src-noconflict/mode-javascript";
import "ace-builds/src-noconflict/mode-java";
import "ace-builds/src-noconflict/mode-python";
import "ace-builds/src-noconflict/theme-monokai";
import "ace-builds/src-noconflict/ext-language_tools";

function App() {
  const [value, setValue] = useState(template);
  const [language, setLanguage] = useState("java");
  const [device, setDevice] = useState({});

  const validateCode = (value: string) => {
    console.log("validate");
  };

  const handleSelect = (e: any) => {
    setLanguage(e.target.value);
  };

  const handleSave = async () => {
    let response = await fetch("http://localhost:6600/savefile", {
      method: "POST",
      headers: {
        "Content-Type": "application/json;charset=utf-8",
      },
      body: JSON.stringify({ code: value }),
    });
  };
  return (
    <div className="main_wrapper">
      <select onChange={(e) => handleSelect(e)} className='select'>
        <option value="javascript">javascript</option>
        <option value="c_cpp">C++</option>
        <option value="python">Python</option>
      </select>
      <AceEditor
        placeholder="Placeholder Text"
        mode={language}
        theme="monokai"
        name="blah2"
        commands={[
          {
            // commands is array of key bindings.
            name: "commandName", //name for the key binding.
            bindKey: { win: "Ctrl-k", mac: "Command-k" }, //key combination used for the command.
            exec: () => validateCode(value), //function to execute when keys are pressed.
          },
        ]}
        onChange={(e) => {
          setValue(e);
        }}
        width={`1000px`}
        height={`1000px`}
        fontSize={14}
        showPrintMargin={true}
        showGutter={true}
        highlightActiveLine={true}
        value={value}
        setOptions={{
          useWorker: true,
          enableBasicAutocompletion: true,
          enableLiveAutocompletion: true,
          enableSnippets: true,
          showLineNumbers: true,
          tabSize: 4,
        }}
      />
      <button
        className="button"
        onClick={() => {
          navigator.bluetooth
            .requestDevice({
              acceptAllDevices: true,
            })
            .then((data) => setDevice(data));
        }}
      >
        View devices
      </button>
      <button
        className="button"
        onClick={() => {
          handleSave();
        }}
      >
        Save sketch
      </button>
    </div>
  );
}

export default App;
const template = `
// the setup function runs once when you press reset or power the board
void setup() {
// initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
`;
