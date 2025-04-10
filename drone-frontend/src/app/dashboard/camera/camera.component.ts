import { Component } from "@angular/core";

@Component({
    selector: 'app-camera', 
    templateUrl: './camera.component.html'
}) export class CameraComponent {
    videoStreamUrl = 'http://localhost:5001/video_feed'
}