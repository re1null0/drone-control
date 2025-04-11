import { Component, OnInit } from '@angular/core';
import { FormGroup, FormControl } from '@angular/forms';
import { HttpClient } from '@angular/common/http';
import { Loader } from '@googlemaps/js-api-loader';

@Component({
  selector: 'app-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.css']
})
export class DashboardComponent implements OnInit {
  constructor(private http: HttpClient) {}

  // Mission form 
  missionFormALL = new FormGroup({
    positions: new FormControl(''),
    altitude: new FormControl(''),
    linearVelocity: new FormControl(''),
    angularVelocity: new FormControl(''),
  });

  selectedMissionType: string = "missionMode";
  missionDisabled: boolean = false;
  surveillanceDisabled: boolean = false;
  hoverDisabled: boolean = false;

  // Google Maps
  public map: any;
  public infoWindow: any;
  public markers: google.maps.Marker[] = [];

  ngOnInit(): void {
    const loader = new Loader({
      apiKey:'', 
      version: 'weekly',
    });

    loader.load().then(() => {
      this.map = new google.maps.Map(
        document.getElementById("map") as HTMLElement,
        {
          center: { lat: -25.344, lng: 131.036 },
          zoom: 19,
        }
      );

      this.infoWindow = new google.maps.InfoWindow();
      this.centerMapToCurrentPosition();

      this.map.addListener("click", (e: any) => {
        this.placeMarkerAndPanTo(e.latLng, this.map);
      });
    });
  }

  get altitudeControl(): FormControl {
    return this.missionFormALL.get('altitude') as FormControl;
  }
  
  get linearVelocityControl(): FormControl {
    return this.missionFormALL.get('linearVelocity') as FormControl;
  }
  
  get angularVelocityControl(): FormControl {
    return this.missionFormALL.get('angularVelocity') as FormControl;
  }
  
  get selectedMissionControl(): FormControl {
    return new FormControl(this.selectedMissionType);
  }
  

  onSubmitArmDrone(arm_status: number) {
    const body = { arm_status };
    const url = "http://localhost:8080/armDrone";
    this.http.post(url, body).toPromise().then((res: any) => console.log(res));
  }

  onSubmitMissionALL() {
    const posArray: any[] = this.missionFormALL.value.positions?.split(" ") || [];
    const body = {
      positions: posArray,
      altitude: this.missionFormALL.value.altitude,
      linear_velocity: this.missionFormALL.value.linearVelocity,
      angular_velocity: this.missionFormALL.value.angularVelocity,
    };
    const url = "http://localhost:8080/uploadALLMission";
    this.http.post(url, body).toPromise().then((res: any) => console.log(res));
  }

  uploadWaypointsToMission(): void {
    if (this.selectedMissionType === "missionMode") {
      let posStr: string = "";
      this.markers.forEach((m) => {
        const lat = m.getPosition()?.lat();
        const lng = m.getPosition()?.lng();
        posStr += `${lat} ${lng} `;
      });
      posStr = posStr.trim();
      this.missionFormALL.patchValue({ positions: posStr });
    }
  }

  placeMarkerAndPanTo(latLng: google.maps.LatLng, map: google.maps.Map): void {
    const marker = new google.maps.Marker({
      position: latLng,
      map: map,
      label: `${this.markers.length + 1}`,
      draggable: true,
    });
    map.panTo(latLng);
    this.markers.push(marker);
    this.checkMissionUploadDisable();
  }

  checkMissionUploadDisable(): void {
    const count = this.markers.length;
    this.missionDisabled = count < 1;
    this.surveillanceDisabled = count < 2;
    this.hoverDisabled = count < 2;
  }

  showMarkers(): void {
    this.setMapOnAll(this.map);
  }

  hideMarkers(): void {
    this.setMapOnAll(null);
  }

  deleteMarkers(): void {
    this.hideMarkers();
    this.markers = [];
    this.checkMissionUploadDisable();
  }

  setMapOnAll(map: google.maps.Map | null): void {
    this.markers.forEach((marker) => marker.setMap(map));
  }

  centerMapToCurrentPosition(): void {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(
        (position: GeolocationPosition) => {
          const pos = {
            lat: position.coords.latitude,
            lng: position.coords.longitude,
          };
          this.infoWindow.setPosition(pos);
          this.infoWindow.setContent("Location found.");
          this.infoWindow.open(this.map);
          this.map.setCenter(pos);
        },
        () => this.handleLocationError(true, this.infoWindow, this.map.getCenter()!)
      );
    } else {
      this.handleLocationError(false, this.infoWindow, this.map.getCenter()!);
    }
  }

  handleLocationError(
    browserHasGeolocation: boolean,
    infoWindow: google.maps.InfoWindow,
    pos: google.maps.LatLng
  ): void {
    infoWindow.setPosition(pos);
    infoWindow.setContent(
      browserHasGeolocation
        ? "Error: The Geolocation service failed."
        : "Error: Your browser doesn't support geolocation."
    );
    infoWindow.open(this.map);
  }
}
