import { Component } from "@angular/core";
import { HttpClient } from "@angular/common/http";

@Component({
    selector: 'app-camera', 
    templateUrl: './camera.component.html'
}) export class CameraComponent {
    videoStreamUrl = 'http://100.77.20.58:5001/video_feed'

    yoloEnabled = false;

    constructor(private http: HttpClient) {}

    toggleYolo() {
        this.http.post<{ yolo_enabled: boolean }>('http://100.77.20.58:5001/toggle_yolo', {})
        .subscribe(res => {
            this.yoloEnabled = res.yolo_enabled;
        });
    }



}
