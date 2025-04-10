import { HttpClient } from '@angular/common/http';
import { Injectable } from '@angular/core';

@Injectable({ providedIn: 'root' })
export class FireSimService {
  baseUrl = 'http://localhost:8000'; // Update to backend URL if deployed

  constructor(private http: HttpClient) {}

  reset(humidity: number, wind: [number, number], numFires: number) {
    return this.http.post(`${this.baseUrl}/reset`, {
      humidity: humidity,
      wind_x: wind[0],
      wind_y: wind[1],
      num_fires: numFires
    });
  }
  

  step(actions: number[][]) {
    return this.http.post<{ done: boolean }>(`${this.baseUrl}/step`, { actions });
  }

  getRender() {
    return this.http.get<{ image: string }>(`${this.baseUrl}/render`);
  }
}
