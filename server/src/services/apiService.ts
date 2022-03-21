
import fs from 'fs';
import avrpizza from  'avr-pizza';
import Avrgirl from 'avrgirl-arduino';
import find from 'local-devices';
import cpplint from 'node-cpplint/lib//index';
import rep from "node-cpplint/lib/reporters"
let reporter = rep.scpec

class ApiService {
    async hexCompiler(code:string) {
     console.log(find)
        fs.writeFileSync("temp.ino", code);
        var pack = {
          sketch: "temp.ino",
          board: "uno",
        };
  
        var options = {
          files: ["D:/custom-ide/server/temp.ino"],
        };

        // cpplint(options, (err,report)=>{
        //   console.log(err)
        //   console.log(report)
        // });
        avrpizza.compile(pack, function(error, hex) {
          if(!error){
            fs.writeFileSync('compiled.hex', hex);
            return hex;
          } else {
            console.log(error)
          }
          })
    }
    async burn(){
      const avrgirl = new Avrgirl({
        board: 'leonardo'
      });
      
      avrgirl.flash('compiled.hex', function (error) {
        if (error) {
          console.error(error);
        } else {
          console.info('done.');
        }
      });
      
    }
};

const apiService = new ApiService();
module.exports = apiService;