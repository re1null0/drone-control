import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';

import { AppComponent } from './app.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { MaterialModule } from './material/material.module';
import { DashboardComponent } from './dashboard/dashboard.component';
import { FormsModule } from '@angular/forms';
import { FireSimComponent } from './dashboard/fire/fire-sim.component';
import { HttpClient, HttpClientModule } from '@angular/common/http';
import { CameraComponent } from './dashboard/camera/camera.component';


@NgModule({
  declarations: [
    AppComponent,
    DashboardComponent,
    FireSimComponent,
    CameraComponent
  ],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    MaterialModule,
    FormsModule,
    HttpClientModule

  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule {}
