<template>
  <v-container fluid justify='center' align='center'>
    <v-row justify='center'>
      <v-col align='center'>
        <v-sheet class="sheet rounded-t-lg mb-1" elevation=0>
          <!-- Sheet 1 -->

          <v-card class="text-secondary" title="LED Controls" color="surface" subtitle="Recent Settings" variant="tonal"
            flat>

          </v-card>
        </v-sheet>

        <v-sheet class="sheet mb-1" elevation=0>
          <!-- Sheet 2 -->

          <v-card class="pt-5" color="surface" variant="tonal">
            <v-slider class="pt-2 bg-surface" append-icon="mdi:mdi-car-light-high" density="compact" thumb-size="16"
              color="secondary" label="Brightness" direction="horizontal" min="0" max="250" step="10" show-ticks
              thumb-label="always" v-model="led.brightness"></v-slider>
          </v-card>
        </v-sheet>

        <v-sheet class="sheet mb-1" elevation=0>
          <!-- Sheet 3 -->
          <v-card class="pt-5" color="surface" variant="tonal">
            <v-slider class="pt-2 bg-surface" append-icon="mdi:mdi-led-on" density="compact" thumb-size="16"
              color="secondary" label="LED Nodes" direction="horizontal" min="1" max="7" step="1" show-ticks
              thumb-label="always" v-model="led.nodes"></v-slider>
          </v-card>
        </v-sheet>

        <v-sheet class="sheet mb-1 pa-2" elevation=0>
          <!-- Sheet 4 -->
          <v-progress-circular rotate="0" size="200" width="15" :model-value="led.nodes * 15" :color="indicatorColor">
            <span class="text-onSurface font-weight-bold">{{ led.nodes }} LED(s)</span>
          </v-progress-circular>
        </v-sheet>
      </v-col>

      <v-col align='center'>
        <!-- Column 2 -->
        <v-color-picker v-model="led.color"></v-color-picker>
      </v-col>
    </v-row>
  </v-container>
</template>

<script setup>
/** JAVASCRIPT HERE */

// IMPORTS
import { ref, reactive, watch, onMounted, onBeforeUnmount, computed } from "vue";
import { useRoute, useRouter } from "vue-router";
import { useMqttStore } from "@/store/mqttStore";
import { storeToRefs } from "pinia";


// VARIABLES
const router = useRouter();
const route = useRoute();

const Mqtt = useMqttStore();
const { payload, payloadTopic } = storeToRefs(Mqtt);

const led = reactive({ "brightness": 255, "leds": 1, "color": { r: 255, g: 255, b: 255, a: 1 } });
let timer, ID = 1000;


// COMPUTED PROPERTIES
const indicatorColor = computed(() => {
  return `rgba(${led.color.r},${led.color.g},${led.color.b},${led.color.a})`
})



// FUNCTIONS
onMounted(() => {
  // THIS FUNCTION IS CALLED AFTER THIS COMPONENT HAS BEEN MOUNTED
  Mqtt.connect();

  setTimeout(() => {
    // Subscribe to each topic
    Mqtt.subscribe("620164974");
    Mqtt.subscribe("620164974_pub");
  }, 3000);
});


onBeforeUnmount(() => {
  // THIS FUNCTION IS CALLED RIGHT BEFORE THIS COMPONENT IS UNMOUNTED
  Mqtt.unsubcribeAll();
});



// WATCHERS
watch(led, (controls) => {
  clearTimeout(ID);

  ID = setTimeout(() => {
    const message = JSON.stringify({ "type": "controls", "brightness": controls.brightness, "leds": controls.nodes, "color": controls.color });
    Mqtt.publish("620164974_sub", message); // Publish to a topic subscribed to by the hardware
  }, 1000)
})

</script>


<style scoped>
/** CSS STYLE HERE */

.sheet {
  color: surface;
  max-width: 800;
  width: 100%;

}
</style>
