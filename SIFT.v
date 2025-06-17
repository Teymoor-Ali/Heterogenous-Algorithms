`timescale 1ns / 1ps

// SIFT Algorithm Implementation for 1080p Images
// Main SIFT processor module
module sift_processor (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] pixel_data,
    input wire pixel_valid,
    output wire [127:0] descriptor_out,
    output wire descriptor_valid,
    output wire [10:0] keypoint_x,
    output wire [10:0] keypoint_y,
    output wire keypoint_valid,
    output wire processing_done,
    output wire ready
);

    // Image dimensions for 1080p
    parameter IMG_WIDTH = 1920;
    parameter IMG_HEIGHT = 1080;
    parameter TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT;
    
    // SIFT parameters
    parameter NUM_OCTAVES = 4;
    parameter NUM_SCALES = 5;
    parameter DESCRIPTOR_SIZE = 128;
    
    // Fixed-point parameters
    parameter FIXED_POINT_WIDTH = 16;
    parameter FIXED_POINT_FRAC = 8;
    
    // State machine states
    typedef enum logic [3:0] {
        IDLE,
        INPUT_IMAGE,
        GAUSSIAN_PYRAMID,
        DOG_COMPUTATION,
        KEYPOINT_DETECTION,
        DESCRIPTOR_GENERATION,
        OUTPUT_RESULTS,
        DONE
    } state_t;
    
    state_t current_state, next_state;
    
    // Memory interfaces
    wire [7:0] image_data_out;
    wire [FIXED_POINT_WIDTH-1:0] gaussian_data_out;
    wire [FIXED_POINT_WIDTH-1:0] dog_data_out;
    
    // Control signals
    reg image_buffer_we;
    reg [20:0] image_buffer_addr;
    reg gaussian_start, dog_start, keypoint_start, descriptor_start;
    wire gaussian_done, dog_done, keypoint_done, descriptor_done;
    
    // Pixel counter for input
    reg [20:0] pixel_counter;
    
    // Image buffer for storing input image
    image_buffer img_buf (
        .clk(clk),
        .we(image_buffer_we),
        .addr(image_buffer_addr),
        .data_in(pixel_data),
        .data_out(image_data_out)
    );
    
    // Gaussian pyramid generator
    gaussian_pyramid_gen gauss_pyramid (
        .clk(clk),
        .rst_n(rst_n),
        .start(gaussian_start),
        .image_data(image_data_out),
        .pyramid_data_out(gaussian_data_out),
        .done(gaussian_done)
    );
    
    // Difference of Gaussians computation
    dog_computer dog_comp (
        .clk(clk),
        .rst_n(rst_n),
        .start(dog_start),
        .gaussian_data(gaussian_data_out),
        .dog_data_out(dog_data_out),
        .done(dog_done)
    );
    
    // Keypoint detector
    keypoint_detector kp_detector (
        .clk(clk),
        .rst_n(rst_n),
        .start(keypoint_start),
        .dog_data(dog_data_out),
        .keypoint_x(keypoint_x),
        .keypoint_y(keypoint_y),
        .keypoint_valid(keypoint_valid),
        .done(keypoint_done)
    );
    
    // Descriptor generator
    descriptor_generator desc_gen (
        .clk(clk),
        .rst_n(rst_n),
        .start(descriptor_start),
        .gaussian_data(gaussian_data_out),
        .keypoint_x(keypoint_x),
        .keypoint_y(keypoint_y),
        .descriptor_out(descriptor_out),
        .descriptor_valid(descriptor_valid),
        .done(descriptor_done)
    );
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
            pixel_counter <= 0;
            image_buffer_addr <= 0;
        end else begin
            current_state <= next_state;
            
            // Handle pixel input
            if (current_state == INPUT_IMAGE && pixel_valid) begin
                pixel_counter <= pixel_counter + 1;
                image_buffer_addr <= image_buffer_addr + 1;
            end
        end
    end
    
    // State machine logic
    always_comb begin
        next_state = current_state;
        image_buffer_we = 0;
        gaussian_start = 0;
        dog_start = 0;
        keypoint_start = 0;
        descriptor_start = 0;
        
        case (current_state)
            IDLE: begin
                if (start) begin
                    next_state = INPUT_IMAGE;
                end
            end
            
            INPUT_IMAGE: begin
                image_buffer_we = pixel_valid;
                if (pixel_counter == TOTAL_PIXELS - 1) begin
                    next_state = GAUSSIAN_PYRAMID;
                end
            end
            
            GAUSSIAN_PYRAMID: begin
                gaussian_start = 1;
                if (gaussian_done) begin
                    next_state = DOG_COMPUTATION;
                end
            end
            
            DOG_COMPUTATION: begin
                dog_start = 1;
                if (dog_done) begin
                    next_state = KEYPOINT_DETECTION;
                end
            end
            
            KEYPOINT_DETECTION: begin
                keypoint_start = 1;
                if (keypoint_done) begin
                    next_state = DESCRIPTOR_GENERATION;
                end
            end
            
            DESCRIPTOR_GENERATION: begin
                descriptor_start = 1;
                if (descriptor_done) begin
                    next_state = OUTPUT_RESULTS;
                end
            end
            
            OUTPUT_RESULTS: begin
                next_state = DONE;
            end
            
            DONE: begin
                // Stay in done state
            end
        endcase
    end
    
    assign ready = (current_state == IDLE);
    assign processing_done = (current_state == DONE);

endmodule

// Image buffer module
module image_buffer (
    input wire clk,
    input wire we,
    input wire [20:0] addr,
    input wire [7:0] data_in,
    output reg [7:0] data_out
);
    
    parameter BUFFER_SIZE = 1920 * 1080;
    reg [7:0] buffer [0:BUFFER_SIZE-1];
    
    always_ff @(posedge clk) begin
        if (we) begin
            buffer[addr] <= data_in;
        end
        data_out <= buffer[addr];
    end
    
endmodule

// Gaussian pyramid generator
module gaussian_pyramid_gen (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] image_data,
    output reg [15:0] pyramid_data_out,
    output reg done
);
    
    parameter IMG_WIDTH = 1920;
    parameter IMG_HEIGHT = 1080;
    parameter NUM_OCTAVES = 4;
    parameter NUM_SCALES = 5;
    
    // Gaussian kernel coefficients (5x5 kernel)
    parameter signed [7:0] GAUSS_KERNEL [0:24] = '{
        1, 4, 6, 4, 1,
        4, 16, 24, 16, 4,
        6, 24, 36, 24, 6,
        4, 16, 24, 16, 4,
        1, 4, 6, 4, 1
    };
    
    typedef enum logic [2:0] {
        G_IDLE,
        G_PROCESS_OCTAVE,
        G_APPLY_GAUSSIAN,
        G_DOWNSAMPLE,
        G_DONE
    } gauss_state_t;
    
    gauss_state_t gauss_state;
    
    reg [2:0] current_octave;
    reg [2:0] current_scale;
    reg [10:0] x_pos, y_pos;
    reg [20:0] pixel_addr;
    
    // Convolution computation
    reg signed [15:0] conv_result;
    reg [4:0] kernel_idx;
    reg [2:0] kernel_x, kernel_y;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gauss_state <= G_IDLE;
            current_octave <= 0;
            current_scale <= 0;
            x_pos <= 0;
            y_pos <= 0;
            done <= 0;
            conv_result <= 0;
        end else begin
            case (gauss_state)
                G_IDLE: begin
                    if (start) begin
                        gauss_state <= G_PROCESS_OCTAVE;
                        current_octave <= 0;
                        current_scale <= 0;
                        done <= 0;
                    end
                end
                
                G_PROCESS_OCTAVE: begin
                    gauss_state <= G_APPLY_GAUSSIAN;
                    x_pos <= 0;
                    y_pos <= 0;
                end
                
                G_APPLY_GAUSSIAN: begin
                    // Apply Gaussian convolution
                    conv_result <= 0;
                    for (int ky = 0; ky < 5; ky++) begin
                        for (int kx = 0; kx < 5; kx++) begin
                            if (x_pos + kx >= 2 && x_pos + kx < IMG_WIDTH - 2 &&
                                y_pos + ky >= 2 && y_pos + ky < IMG_HEIGHT - 2) begin
                                conv_result <= conv_result + 
                                    (image_data * GAUSS_KERNEL[ky*5 + kx]) >> 8;
                            end
                        end
                    end
                    
                    pyramid_data_out <= conv_result;
                    
                    // Move to next pixel
                    if (x_pos < IMG_WIDTH - 1) begin
                        x_pos <= x_pos + 1;
                    end else begin
                        x_pos <= 0;
                        if (y_pos < IMG_HEIGHT - 1) begin
                            y_pos <= y_pos + 1;
                        end else begin
                            gauss_state <= G_DOWNSAMPLE;
                        end
                    end
                end
                
                G_DOWNSAMPLE: begin
                    // Move to next scale or octave
                    if (current_scale < NUM_SCALES - 1) begin
                        current_scale <= current_scale + 1;
                        gauss_state <= G_PROCESS_OCTAVE;
                    end else begin
                        current_scale <= 0;
                        if (current_octave < NUM_OCTAVES - 1) begin
                            current_octave <= current_octave + 1;
                            gauss_state <= G_PROCESS_OCTAVE;
                        end else begin
                            gauss_state <= G_DONE;
                        end
                    end
                end
                
                G_DONE: begin
                    done <= 1;
                end
            endcase
        end
    end
    
endmodule

// Difference of Gaussians computer
module dog_computer (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [15:0] gaussian_data,
    output reg [15:0] dog_data_out,
    output reg done
);
    
    parameter NUM_OCTAVES = 4;
    parameter NUM_SCALES = 5;
    parameter IMG_WIDTH = 1920;
    parameter IMG_HEIGHT = 1080;
    
    typedef enum logic [1:0] {
        DOG_IDLE,
        DOG_COMPUTE,
        DOG_DONE
    } dog_state_t;
    
    dog_state_t dog_state;
    
    reg [2:0] current_octave;
    reg [2:0] current_scale;
    reg [15:0] prev_gaussian_data;
    reg [20:0] pixel_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dog_state <= DOG_IDLE;
            current_octave <= 0;
            current_scale <= 0;
            done <= 0;
            pixel_counter <= 0;
        end else begin
            case (dog_state)
                DOG_IDLE: begin
                    if (start) begin
                        dog_state <= DOG_COMPUTE;
                        current_octave <= 0;
                        current_scale <= 1; // Start from scale 1
                        pixel_counter <= 0;
                        done <= 0;
                    end
                end
                
                DOG_COMPUTE: begin
                    // Compute difference between consecutive scales
                    dog_data_out <= gaussian_data - prev_gaussian_data;
                    prev_gaussian_data <= gaussian_data;
                    
                    pixel_counter <= pixel_counter + 1;
                    
                    // Check if we've processed all pixels for current scale
                    if (pixel_counter == (IMG_WIDTH * IMG_HEIGHT) - 1) begin
                        pixel_counter <= 0;
                        if (current_scale < NUM_SCALES - 1) begin
                            current_scale <= current_scale + 1;
                        end else begin
                            current_scale <= 1;
                            if (current_octave < NUM_OCTAVES - 1) begin
                                current_octave <= current_octave + 1;
                            end else begin
                                dog_state <= DOG_DONE;
                            end
                        end
                    end
                end
                
                DOG_DONE: begin
                    done <= 1;
                end
            endcase
        end
    end
    
endmodule

// Keypoint detector module
module keypoint_detector (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [15:0] dog_data,
    output reg [10:0] keypoint_x,
    output reg [10:0] keypoint_y,
    output reg keypoint_valid,
    output reg done
);
    
    parameter IMG_WIDTH = 1920;
    parameter IMG_HEIGHT = 1080;
    parameter CONTRAST_THRESHOLD = 16'h0A00; // 0.03 in fixed point
    parameter EDGE_THRESHOLD = 16'h2800;     // 10.0 in fixed point
    
    typedef enum logic [2:0] {
        KP_IDLE,
        KP_SCAN,
        KP_CHECK_EXTREMA,
        KP_VALIDATE,
        KP_DONE
    } kp_state_t;
    
    kp_state_t kp_state;
    
    reg [10:0] scan_x, scan_y;
    reg [15:0] center_value;
    reg [15:0] neighbor_values [0:25]; // 3x3x3 neighborhood
    reg is_maxima, is_minima;
    reg [4:0] neighbor_idx;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_state <= KP_IDLE;
            scan_x <= 1;
            scan_y <= 1;
            keypoint_valid <= 0;
            done <= 0;
        end else begin
            case (kp_state)
                KP_IDLE: begin
                    if (start) begin
                        kp_state <= KP_SCAN;
                        scan_x <= 1;
                        scan_y <= 1;
                        keypoint_valid <= 0;
                        done <= 0;
                    end
                end
                
                KP_SCAN: begin
                    center_value <= dog_data;
                    kp_state <= KP_CHECK_EXTREMA;
                    neighbor_idx <= 0;
                end
                
                KP_CHECK_EXTREMA: begin
                    // Check if current pixel is local extrema
                    // This is simplified - in practice, need to check 3x3x3 neighborhood
                    is_maxima <= 1;
                    is_minima <= 1;
                    
                    for (int i = 0; i < 26; i++) begin
                        if (center_value <= neighbor_values[i]) begin
                            is_maxima <= 0;
                        end
                        if (center_value >= neighbor_values[i]) begin
                            is_minima <= 0;
                        end
                    end
                    
                    kp_state <= KP_VALIDATE;
                end
                
                KP_VALIDATE: begin
                    // Validate keypoint based on contrast and edge response
                    if ((is_maxima || is_minima) && 
                        (center_value > CONTRAST_THRESHOLD || center_value < -CONTRAST_THRESHOLD)) begin
                        keypoint_x <= scan_x;
                        keypoint_y <= scan_y;
                        keypoint_valid <= 1;
                    end else begin
                        keypoint_valid <= 0;
                    end
                    
                    // Move to next pixel
                    if (scan_x < IMG_WIDTH - 2) begin
                        scan_x <= scan_x + 1;
                        kp_state <= KP_SCAN;
                    end else begin
                        scan_x <= 1;
                        if (scan_y < IMG_HEIGHT - 2) begin
                            scan_y <= scan_y + 1;
                            kp_state <= KP_SCAN;
                        end else begin
                            kp_state <= KP_DONE;
                        end
                    end
                end
                
                KP_DONE: begin
                    done <= 1;
                end
            endcase
        end
    end
    
endmodule

// Descriptor generator module
module descriptor_generator (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [15:0] gaussian_data,
    input wire [10:0] keypoint_x,
    input wire [10:0] keypoint_y,
    output reg [127:0] descriptor_out,
    output reg descriptor_valid,
    output reg done
);
    
    parameter DESCRIPTOR_SIZE = 128;
    parameter PATCH_SIZE = 16;
    parameter HIST_BINS = 8;
    
    typedef enum logic [2:0] {
        DESC_IDLE,
        DESC_COMPUTE_GRADIENTS,
        DESC_BUILD_HISTOGRAM,
        DESC_NORMALIZE,
        DESC_OUTPUT,
        DESC_DONE
    } desc_state_t;
    
    desc_state_t desc_state;
    
    reg [15:0] gradients_mag [0:255];   // 16x16 patch
    reg [7:0] gradients_orient [0:255]; // 16x16 patch
    reg [15:0] histogram [0:127];       // 4x4x8 = 128 bins
    reg [7:0] patch_x, patch_y;
    reg [6:0] hist_idx;
    
    // Gradient computation
    reg signed [15:0] dx, dy;
    reg [15:0] magnitude;
    reg [7:0] orientation;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            desc_state <= DESC_IDLE;
            patch_x <= 0;
            patch_y <= 0;
            descriptor_valid <= 0;
            done <= 0;
            hist_idx <= 0;
        end else begin
            case (desc_state)
                DESC_IDLE: begin
                    if (start) begin
                        desc_state <= DESC_COMPUTE_GRADIENTS;
                        patch_x <= 0;
                        patch_y <= 0;
                        descriptor_valid <= 0;
                        done <= 0;
                    end
                end
                
                DESC_COMPUTE_GRADIENTS: begin
                    // Compute gradient magnitude and orientation
                    // This is simplified - actual implementation needs neighboring pixels
                    dx <= gaussian_data; // Placeholder
                    dy <= gaussian_data; // Placeholder
                    
                    magnitude <= $sqrt(dx*dx + dy*dy);
                    orientation <= $atan2(dy, dx) * 256 / (2 * 3.14159);
                    
                    gradients_mag[patch_y * PATCH_SIZE + patch_x] <= magnitude;
                    gradients_orient[patch_y * PATCH_SIZE + patch_x] <= orientation;
                    
                    // Move to next pixel in patch
                    if (patch_x < PATCH_SIZE - 1) begin
                        patch_x <= patch_x + 1;
                    end else begin
                        patch_x <= 0;
                        if (patch_y < PATCH_SIZE - 1) begin
                            patch_y <= patch_y + 1;
                        end else begin
                            desc_state <= DESC_BUILD_HISTOGRAM;
                            patch_x <= 0;
                            patch_y <= 0;
                        end
                    end
                end
                
                DESC_BUILD_HISTOGRAM: begin
                    // Build 4x4 grid of 8-bin histograms
                    // Simplified implementation
                    for (int i = 0; i < 128; i++) begin
                        histogram[i] <= 0;
                    end
                    
                    // Accumulate gradients into histogram bins
                    for (int y = 0; y < 16; y++) begin
                        for (int x = 0; x < 16; x++) begin
                            automatic int grid_x = x / 4;
                            automatic int grid_y = y / 4;
                            automatic int bin = gradients_orient[y*16 + x] / 32;
                            automatic int hist_index = (grid_y * 4 + grid_x) * 8 + bin;
                            
                            if (hist_index < 128) begin
                                histogram[hist_index] <= histogram[hist_index] + 
                                                       gradients_mag[y*16 + x];
                            end
                        end
                    end
                    
                    desc_state <= DESC_NORMALIZE;
                end
                
                DESC_NORMALIZE: begin
                    // Normalize descriptor vector
                    // Simplified - actual implementation needs L2 normalization
                    desc_state <= DESC_OUTPUT;
                end
                
                DESC_OUTPUT: begin
                    // Output descriptor
                    descriptor_out <= {histogram[0], histogram[1], histogram[2], histogram[3],
                                     histogram[4], histogram[5], histogram[6], histogram[7]};
                    descriptor_valid <= 1;
                    desc_state <= DESC_DONE;
                end
                
                DESC_DONE: begin
                    done <= 1;
                end
            endcase
        end
    end
    
endmodule

// Testbench for SIFT processor
module sift_processor_tb;
    
    reg clk, rst_n, start;
    reg [7:0] pixel_data;
    reg pixel_valid;
    wire [127:0] descriptor_out;
    wire descriptor_valid;
    wire [10:0] keypoint_x, keypoint_y;
    wire keypoint_valid;
    wire processing_done, ready;
    
    // Clock generation
    always #5 clk = ~clk;
    
    // Instantiate SIFT processor
    sift_processor uut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .pixel_data(pixel_data),
        .pixel_valid(pixel_valid),
        .descriptor_out(descriptor_out),
        .descriptor_valid(descriptor_valid),
        .keypoint_x(keypoint_x),
        .keypoint_y(keypoint_y),
        .keypoint_valid(keypoint_valid),
        .processing_done(processing_done),
        .ready(ready)
    );
    
    initial begin
        clk = 0;
        rst_n = 0;
        start = 0;
        pixel_data = 0;
        pixel_valid = 0;
        
        // Reset
        #100;
        rst_n = 1;
        #50;
        
        // Start processing
        start = 1;
        #10;
        start = 0;
        
        // Feed test image data
        for (int i = 0; i < 1920*1080; i++) begin
            pixel_data = $random % 256;
            pixel_valid = 1;
            #10;
        end
        pixel_valid = 0;
        
        // Wait for processing to complete
        wait(processing_done);
        
        $display("SIFT processing completed");
        $finish;
    end
    
    // Monitor outputs
    always @(posedge clk) begin
        if (keypoint_valid) begin
            $display("Keypoint detected at (%d, %d)", keypoint_x, keypoint_y);
        end
        if (descriptor_valid) begin
            $display("Descriptor generated: %h", descriptor_out);
        end
    end
    
endmodule