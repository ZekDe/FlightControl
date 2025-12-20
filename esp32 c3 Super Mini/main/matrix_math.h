/**
 * @file matrix_math.h
 * @brief General-purpose matrix mathematics library for embedded systems
 * @details Provides matrix operations: add, subtract, multiply, transpose,
 *          inverse, identity, and utility functions.
 *          Designed for EKF and similar algorithms on MCUs with FPU.
 * 
 * @note Maximum matrix size is configurable via MATRIX_MAX_SIZE
 * @note Uses float precision (single precision FPU)
 */

#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#ifndef MATRIX_MAX_SIZE
#define MATRIX_MAX_SIZE     10      /**< Maximum matrix dimension (rows or cols) */
#endif

#define MATRIX_EPSILON      1e-6f   /**< Threshold for floating point comparisons */

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define MATRIX_OK                   0   /**< Operation successful */
#define MATRIX_ERR_NULL_PTR        -1   /**< Null pointer passed */
#define MATRIX_ERR_DIM_MISMATCH    -2   /**< Matrix dimensions incompatible */
#define MATRIX_ERR_SINGULAR        -3   /**< Matrix is singular, cannot invert */
#define MATRIX_ERR_OUT_OF_BOUNDS   -4   /**< Index out of bounds */
#define MATRIX_ERR_INVALID_SIZE    -5   /**< Invalid matrix size */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/

/**
 * @brief Matrix structure with embedded data storage
 * @note Data is stored in row-major order: data[row * cols + col]
 */
typedef struct {
    uint8_t rows;                               /**< Number of rows */
    uint8_t cols;                               /**< Number of columns */
    float data[MATRIX_MAX_SIZE * MATRIX_MAX_SIZE]; /**< Matrix data (row-major) */
} Matrix_t;

/*******************************************************************************
 * Matrix Creation and Initialization
 ******************************************************************************/

/**
 * @brief Initialize a matrix with zeros
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  rows   Number of rows
 * @param[in]  cols   Number of columns
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixInit(Matrix_t *mat, uint8_t rows, uint8_t cols);

/**
 * @brief Create identity matrix
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  size   Matrix dimension (creates size x size identity)
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixEye(Matrix_t *mat, uint8_t size);

/**
 * @brief Fill matrix with a constant value
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  value  Value to fill
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixFill(Matrix_t *mat, float value);

/**
 * @brief Set matrix data from array (row-major order)
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  data   Source data array (must have rows*cols elements)
 * @param[in]  rows   Number of rows
 * @param[in]  cols   Number of columns
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixSetData(Matrix_t *mat, const float *data, uint8_t rows, uint8_t cols);

/**
 * @brief Create diagonal matrix from vector
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  diag   Diagonal values array
 * @param[in]  size   Number of diagonal elements
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixDiag(Matrix_t *mat, const float *diag, uint8_t size);

/*******************************************************************************
 * Element Access
 ******************************************************************************/

/**
 * @brief Get element at (row, col)
 * @param[in]  mat    Pointer to matrix structure
 * @param[in]  row    Row index (0-based)
 * @param[in]  col    Column index (0-based)
 * @param[out] value  Pointer to store retrieved value
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixGet(const Matrix_t *mat, uint8_t row, uint8_t col, float *value);

/**
 * @brief Set element at (row, col)
 * @param[out] mat    Pointer to matrix structure
 * @param[in]  row    Row index (0-based)
 * @param[in]  col    Column index (0-based)
 * @param[in]  value  Value to set
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixSet(Matrix_t *mat, uint8_t row, uint8_t col, float value);

/*******************************************************************************
 * Basic Matrix Operations
 ******************************************************************************/

/**
 * @brief Matrix addition: result = A + B
 * @param[in]  A       First matrix
 * @param[in]  B       Second matrix
 * @param[out] result  Result matrix (can be same as A or B)
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixAdd(const Matrix_t *A, const Matrix_t *B, Matrix_t *result);

/**
 * @brief Matrix subtraction: result = A - B
 * @param[in]  A       First matrix
 * @param[in]  B       Second matrix
 * @param[out] result  Result matrix (can be same as A or B)
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixSub(const Matrix_t *A, const Matrix_t *B, Matrix_t *result);

/**
 * @brief Matrix multiplication: result = A * B
 * @param[in]  A       First matrix (m x n)
 * @param[in]  B       Second matrix (n x p)
 * @param[out] result  Result matrix (m x p), must NOT overlap with A or B
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixMul(const Matrix_t *A, const Matrix_t *B, Matrix_t *result);

/**
 * @brief Scalar multiplication: result = scalar * A
 * @param[in]  A       Input matrix
 * @param[in]  scalar  Scalar value
 * @param[out] result  Result matrix (can be same as A)
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixScale(const Matrix_t *A, float scalar, Matrix_t *result);

/**
 * @brief Matrix transpose: result = A^T
 * @param[in]  A       Input matrix (m x n)
 * @param[out] result  Result matrix (n x m), must NOT be same as A
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixTranspose(const Matrix_t *A, Matrix_t *result);

/*******************************************************************************
 * Advanced Matrix Operations
 ******************************************************************************/

/**
 * @brief Matrix inverse using Gauss-Jordan elimination: result = A^(-1)
 * @param[in]  A       Input matrix (must be square)
 * @param[out] result  Result matrix, must NOT be same as A
 * @return MATRIX_OK on success, MATRIX_ERR_SINGULAR if not invertible
 */
int8_t matrixInverse(const Matrix_t *A, Matrix_t *result);

/**
 * @brief Compute matrix determinant
 * @param[in]  A       Input matrix (must be square)
 * @param[out] det     Pointer to store determinant value
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixDeterminant(const Matrix_t *A, float *det);

/**
 * @brief Compute matrix trace (sum of diagonal elements)
 * @param[in]  A       Input matrix (must be square)
 * @param[out] trace   Pointer to store trace value
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixTrace(const Matrix_t *A, float *trace);

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

/**
 * @brief Copy matrix: dest = src
 * @param[in]  src     Source matrix
 * @param[out] dest    Destination matrix
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixCopy(const Matrix_t *src, Matrix_t *dest);

/**
 * @brief Check if two matrices are equal (within epsilon tolerance)
 * @param[in]  A       First matrix
 * @param[in]  B       Second matrix
 * @param[out] equal   1 if equal, 0 if not
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixEqual(const Matrix_t *A, const Matrix_t *B, uint8_t *equal);

/**
 * @brief Compute Frobenius norm of matrix: ||A||_F = sqrt(sum(a_ij^2))
 * @param[in]  A       Input matrix
 * @param[out] norm    Pointer to store norm value
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixNorm(const Matrix_t *A, float *norm);

/**
 * @brief Create linearly spaced vector
 * @param[out] mat     Output matrix (will be 1 x n)
 * @param[in]  start   Starting value
 * @param[in]  end     Ending value
 * @param[in]  n       Number of points
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixLinspace(Matrix_t *mat, float start, float end, uint8_t n);

/**
 * @brief Create linearly incrementing vector: [start, start+step, start+2*step, ...]
 * @param[out] mat     Output matrix (will be 1 x n)
 * @param[in]  start   Starting value
 * @param[in]  step    Increment step
 * @param[in]  n       Number of points
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixLinIncrement(Matrix_t *mat, float start, float step, uint8_t n);

/**
 * @brief Extract submatrix
 * @param[in]  A          Source matrix
 * @param[out] result     Destination matrix
 * @param[in]  start_row  Starting row (0-based)
 * @param[in]  start_col  Starting column (0-based)
 * @param[in]  num_rows   Number of rows to extract
 * @param[in]  num_cols   Number of columns to extract
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixSubmatrix(const Matrix_t *A, Matrix_t *result,
                       uint8_t start_row, uint8_t start_col,
                       uint8_t num_rows, uint8_t num_cols);

/**
 * @brief Set submatrix within larger matrix
 * @param[out] A          Destination matrix
 * @param[in]  sub        Source submatrix
 * @param[in]  start_row  Starting row (0-based)
 * @param[in]  start_col  Starting column (0-based)
 * @return MATRIX_OK on success, error code otherwise
 */
int8_t matrixSetSubmatrix(Matrix_t *A, const Matrix_t *sub,
                          uint8_t start_row, uint8_t start_col);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_MATH_H */
