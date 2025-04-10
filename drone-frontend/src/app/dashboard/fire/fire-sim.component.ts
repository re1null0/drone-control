import { Component, ChangeDetectorRef } from "@angular/core";
import { FireSimService } from "./fire-sim.service";
import { interval, Subscription } from "rxjs";



@Component({
    selector: 'app-fire-sim',
    templateUrl: './fire-sim.component.html',
    styleUrls: ['/fire-sim.component.css']
  
})
export class FireSimComponent {
  humidity = 0.5;
  windDirection = 'NE';
  numFires = 0;
  actionInput = '[[5,5],[6,6]]';
  image: string = '';
  running = false;
  simLoop: Subscription | null = null;

  directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'];

  constructor(private fireSim: FireSimService, private cdRef: ChangeDetectorRef) {}

  ngOnInit(): void {
    this.reset();
  }

  reset() {
    const wind = this.getWindVector(this.windDirection);
      this.fireSim.reset(this.humidity, wind, this.numFires).subscribe(() => {
        this.loadImage();
      });
  }
  
  start() {
    if (this.running) return;
    this.running = true;

    const wind = this.getWindVector(this.windDirection);

    this.fireSim.reset(this.humidity, wind, this.numFires).subscribe(() => {
        this.loadImage();

        this.simLoop = interval(100).subscribe(() => {
        let actions;
        try {
            actions = JSON.parse(this.actionInput);
        } catch {
            this.stop();
            alert("Invalid JSON in action input.");
            return;
        }

        this.fireSim.step(actions).subscribe(res => {
            this.loadImage();
            if (res.done) this.stop();  // stop when simulation ends
        });
        });
    });
  }

  stop() {
    if (this.simLoop) this.simLoop.unsubscribe();
    this.running = false;
   }
   
   loadImage() {
    this.fireSim.getRender().subscribe(res => {
      this.image = res.image;
      this.cdRef.detectChanges();
    });
  }

  getWindVector(dir: string): [number, number] {
    const map: Record<string, [number, number]> = {
      N: [0, 1],
      NE: [1, 1],
      E: [1, 0],
      SE: [1, -1],
      S: [0, -1],
      SW: [-1, -1],
      W: [-1, 0],
      NW: [-1, 1]
    };
    return map[dir];
  }
}
